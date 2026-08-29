%% s02_balance — live PID tuning console. The loop runs on the ESP32.
%  Lay the robot down, run this, then lift it upright. It arms itself when
%  the balance angle enters the capture window and disarms when it leaves.
%  Sliders change gains live. STOP button, 'q', or Ctrl+C to finish.

clear; clc; close all;

PORT      = "COM4";     % or "192.168.4.1" for WiFi
DURATION  = 300;
U_MAX     = 3.0;
TILT_MAX  = deg2rad(30);
CAPTURE   = deg2rad(10);
CAPTURE_N = 5;

Kp0 = 8.0;  Ki0 = 0.0;  Kd0 = 0.5;  Trim0 = 0.0;
KpMax = 40; KiMax = 20; KdMax = 5;  TrimMax = 10;

lnk = SbrLink(PORT);
cleanupObj = onCleanup(@() delete(lnk));

d = [];  tw = tic;
while toc(tw) < 1.5
    dn = lnk.readLatest();
    if ~isempty(dn), d = dn; end
end
assert(~isempty(d), 'No telemetry on %s. Run s00_motor_check first.', PORT);
assert(d.foc_hz >= 0, 'initFOC() failed on the motor node. See s00_motor_check.');
assert(d.imu_ok, 'imu_ok = 0. The IMU node is not delivering attitude.');
fprintf('link ok: foc %.0f Hz, imu %.0f Hz\n', d.foc_hz, d.imu_hz);

lnk.trim(Trim0);
lnk.limits(U_MAX, TILT_MAX);
lnk.setpoint(0);
lnk.gains(Kp0, Ki0, Kd0);
lnk.mode(2);

fig = figure('Name','s02 — balance tuning (Q = stop)','NumberTitle','off', ...
             'Position',[120 100 1020 780]);
setappdata(fig,'stop',false);
fig.KeyPressFcn = @(s,e) setappdata(s,'stop', strcmpi(e.Key,'q'));

names = {'Kp','Ki','Kd','Trim'};
maxes = [KpMax KiMax KdMax TrimMax];
mins  = [0 0 0 -TrimMax];
vals  = [Kp0 Ki0 Kd0 Trim0];
sl = gobjects(4,1); tx = gobjects(4,1);
for i = 1:4
    uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.03 0.955-0.042*i 0.05 0.028],'String',names{i}, ...
        'FontWeight','bold','HorizontalAlignment','left');
    sl(i) = uicontrol(fig,'Style','slider','Units','normalized', ...
        'Position',[0.09 0.955-0.042*i 0.58 0.028], ...
        'Min',mins(i),'Max',maxes(i),'Value',vals(i));
    tx(i) = uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[0.68 0.955-0.042*i 0.09 0.028], ...
        'String',sprintf('%.2f',vals(i)),'HorizontalAlignment','left');
end

hState = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.79 0.905 0.18 0.055],'String','WAITING', ...
    'FontSize',13,'FontWeight','bold','BackgroundColor',[0.90 0.85 0.55]);
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.79 0.795 0.18 0.085],'String','STOP','FontSize',14, ...
    'FontWeight','bold','BackgroundColor',[0.85 0.35 0.25], ...
    'Callback',@(s,e) setappdata(fig,'stop',true));

axA = axes(fig,'Position',[0.08 0.44 0.87 0.30]);
hA  = animatedline(axA,'Color',[0.85 0.45 0.05],'LineWidth',1.3);
ylabel(axA,'angle [deg]'); grid(axA,'on'); hold(axA,'on');
yline(axA,0,'-','Color',[0.6 0.6 0.6]);
yline(axA,[-1 1]*rad2deg(CAPTURE),':','Color',[0.45 0.55 0.45]);

axU = axes(fig,'Position',[0.08 0.07 0.87 0.29]);
hU  = animatedline(axU,'Color',[0.05 0.45 0.50],'LineWidth',1.3);
ylabel(axU,'command u'); xlabel(axU,'t [s]'); grid(axU,'on');
yline(axU,[-U_MAX U_MAX],'--','Color',[0.65 0.25 0.15]);

fprintf('\nLay the robot down. Lift it upright to start balancing.\n\n');

N = ceil(DURATION*60);
LOG = struct('t',nan(N,1),'angle',nan(N,1),'rate',nan(N,1),'u',nan(N,1), ...
             'pos',nan(N,1),'kp',nan(N,1),'ki',nan(N,1),'kd',nan(N,1));

k = 0; nWin = 0; nDraw = 0; balancing = false;
last = [Kp0 Ki0 Kd0]; trimNow = Trim0;
t0 = tic; tTx = tic; nFalls = 0;

while toc(t0) < DURATION
    if ~isvalid(fig) || getappdata(fig,'stop'), break; end

    cur = [sl(1).Value sl(2).Value sl(3).Value];
    if any(abs(cur - last) > 1e-3)
        lnk.gains(cur(1), cur(2), cur(3));
        last = cur; tTx = tic;
    end
    curTrim = deg2rad(sl(4).Value);
    if abs(curTrim - trimNow) > 1e-4
        lnk.trim(curTrim);
        trimNow = curTrim; tTx = tic;
    end
    for i = 1:4, tx(i).String = sprintf('%.2f', sl(i).Value); end

    if toc(tTx) > 0.1
        lnk.heartbeat();
        tTx = tic;
    end

    d = lnk.readLatest();
    if isempty(d), drawnow limitrate; continue; end

    if ~d.imu_ok
        fprintf('!! imu_ok = 0 — the IMU node stopped delivering. Stopping.\n');
        break
    end

    t   = toc(t0);
    ang = d.angle - trimNow;

    if ~balancing
        if abs(ang) < CAPTURE
            nWin = nWin + 1;
            if nWin >= CAPTURE_N
                lnk.arm(true);
                balancing = true; nWin = 0;
                hState.String = 'BALANCING';
                hState.BackgroundColor = [0.55 0.80 0.55];
                fprintf('  captured at t = %5.1f s\n', t);
            end
        else
            nWin = 0;
        end
    else
        if d.tilt_fault || ~d.armed
            lnk.arm(false);
            balancing = false; nWin = 0; nFalls = nFalls + 1;
            hState.String = 'WAITING';
            hState.BackgroundColor = [0.90 0.85 0.55];
            fprintf('  lost it at t = %5.1f s (%.0f deg) — lift it back up\n', ...
                    t, rad2deg(ang));
        else
            k = k + 1;
            LOG.t(k)=t; LOG.angle(k)=ang; LOG.rate(k)=d.rate; LOG.u(k)=d.u0;
            LOG.pos(k)=(d.pos0+d.pos1)/2;
            LOG.kp(k)=last(1); LOG.ki(k)=last(2); LOG.kd(k)=last(3);
            addpoints(hU, t, d.u0);
        end
    end

    addpoints(hA, t, rad2deg(ang));
    nDraw = nDraw + 1;
    if mod(nDraw, 10) == 0
        xlim(axA,[max(0,t-12) max(12,t)]); xlim(axU,[max(0,t-12) max(12,t)]);
        drawnow limitrate;
    end
end

lnk.stop();

v = ~isnan(LOG.t);
T = struct2table(structfun(@(x) x(v), LOG, 'UniformOutput', false));

fprintf('\n=== Result ===\n');
fprintf('  falls               : %d\n', nFalls);
if height(T) == 0
    fprintf(2, '  Never captured — the balance angle never came within %.0f deg of trim.\n', ...
            rad2deg(CAPTURE));
    fprintf(2, '  Check that s01_imu_axis picked the right channel.\n');
    return
end

dt = diff(T.t);
fprintf('  time balancing      : %.1f s (%d samples)\n', sum(dt(dt < 0.2)), height(T));
fprintf('  longest single run  : %.1f s\n', maxRun(T.t));
fprintf('  final gains         : Kp=%.2f  Ki=%.2f  Kd=%.2f  trim=%.2f deg\n', ...
        last, rad2deg(trimNow));
fprintf('  angle RMS           : %.2f deg\n', rad2deg(rms(T.angle)));
fprintf('  max |angle|         : %.2f deg\n', rad2deg(max(abs(T.angle))));
fprintf('  command RMS         : %.3f  (saturated %.1f%%)\n', ...
        rms(T.u), 100*mean(abs(T.u) >= U_MAX*0.99));
fprintf('  wheel drift         : %.2f rad\n', T.pos(end)-T.pos(1));

if ~exist('../logs','dir'), mkdir('../logs'); end
fname = sprintf('../logs/s02_balance_%s.mat', ...
                string(datetime('now','Format','yyyyMMdd_HHmmss')));
save(fname,'T','U_MAX','last','trimNow','nFalls');
fprintf('  saved               : %s\n', fname);

fprintf(['\nTuning order:\n' ...
         '  1. Ki=0, Kd=0. Raise Kp until it reacts briskly and starts to\n' ...
         '     oscillate fast. Note that value.\n' ...
         '  2. Drop Kp to ~60%% of it, raise Kd until the oscillation goes.\n' ...
         '  3. Nudge Trim until it stops creeping in one direction.\n' ...
         '  4. Add Ki (0.5..2) last, only to remove steady-state lean.\n']);

function s = maxRun(t)
    gaps = [0; find(diff(t) >= 0.2); numel(t)];
    s = 0;
    for i = 1:numel(gaps)-1
        seg = t(gaps(i)+1 : gaps(i+1));
        if numel(seg) > 1, s = max(s, seg(end) - seg(1)); end
    end
end
