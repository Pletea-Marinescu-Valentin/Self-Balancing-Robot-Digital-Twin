%% s02_balance — live tuning console. Both control loops run on the ESP32.
%  Lay the robot down, run this, then lift it upright. It arms itself when
%  the balance angle enters the capture window and disarms when it leaves.
%  Every gain is both a slider and a typed box: drag to explore, type to
%  land on a value exactly. A typed number outside the slider's range widens
%  the slider rather than being clamped away.
%
%  The run has no time limit. It ends on the STOP button, 'q', by closing the
%  window, or with Ctrl+C. The log grows to fit however long that takes.
%
%  Two loops, tuned in order:
%    inner (Kp Ki Kd Trim) holds the tilt angle at 200 Hz
%    outer (Kv Kvi Lean)   holds the wheels still at 40 Hz, by choosing the
%                          tilt angle the inner loop is told to hold
%
%  The outer loop is why the robot stays on the table. Tune the inner one
%  first with Kv = Kvi = 0; a wheel loop wrapped around a badly damped
%  balance loop tells you nothing.

clear; clc; close all;

PORT      = "192.168.4.1";     % or "COM4" for USB
U_MAX     = 3.0;
TILT_MAX  = deg2rad(30);
CAPTURE   = deg2rad(10);
CAPTURE_N = 5;

% Inner loop: angle. Outer loop: wheels. Kv and Kvi are in degrees of lean
% per unit of wheel state, which is the only form of them worth reading on a
% slider; SbrLink converts to radians on the wire.
Kp0 = 8.0;  Ki0  = 0.0;   Kd0  = 0.5;   Trim0 = 0.0;
Kv0 = 0.6;  Kvi0 = 0.15;  Lean0 = 3.0;  YawD0 = 0.05;  Ufric0 = 0.0;

KpMax = 40;  KiMax  = 20;   KdMax  = 5;    TrimMax = 10;
KvMax = 3;   KviMax = 1;    LeanMax = 8;   YawDMax = 0.5;  UfricMax = 1.0;

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
lnk.outer(deg2rad(Kv0), deg2rad(Kvi0), deg2rad(Lean0));
lnk.yaw(0, YawD0);
lnk.friction(Ufric0, 0.5);
lnk.mode(2);

fig = figure('Name','s02 — balance tuning (Q = stop)','NumberTitle','off', ...
             'Position',[100 40 1120 900]);
setappdata(fig,'stop',false);
fig.KeyPressFcn = @(s,e) setappdata(s,'stop', strcmpi(e.Key,'q'));

% Two columns, because the two loops are tuned at different times and it
% should be obvious at a glance which slider belongs to which.
names = {'Kp','Ki','Kd','Trim','Kv','Kvi','Lean','YawD','Ufric'};
mins  = [0 0 0 -TrimMax  -KvMax -KviMax 0 0 0];
maxes = [KpMax KiMax KdMax TrimMax  KvMax KviMax LeanMax YawDMax UfricMax];
vals  = [Kp0 Ki0 Kd0 Trim0  Kv0 Kvi0 Lean0 YawD0 Ufric0];
col   = [1 1 1 1  2 2 2 2 2];      % 1 = inner loop, 2 = outer loop
row   = [1 2 3 4  1 2 3 4 5];

uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.03 0.955 0.30 0.028],'String','BALANCE  (inner, 200 Hz)', ...
    'FontWeight','bold','HorizontalAlignment','left');
uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.44 0.955 0.30 0.028],'String','WHEELS  (outer, 40 Hz)', ...
    'FontWeight','bold','HorizontalAlignment','left');

n  = numel(names);
sl = gobjects(n,1); tx = gobjects(n,1);
x0 = [0.03 0.44];
for i = 1:n
    y = 0.918 - 0.034*(row(i)-1);
    b = x0(col(i));
    uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[b y 0.055 0.026],'String',names{i}, ...
        'FontWeight','bold','HorizontalAlignment','left');
    sl(i) = uicontrol(fig,'Style','slider','Units','normalized', ...
        'Position',[b+0.055 y 0.235 0.026], ...
        'Min',mins(i),'Max',maxes(i),'Value',vals(i));
    % Editable, not a readout. Late in a tune the useful move is "make it
    % exactly 12.4", and a slider cannot express that.
    tx(i) = uicontrol(fig,'Style','edit','Units','normalized', ...
        'Position',[b+0.295 y 0.068 0.026], ...
        'String',sprintf('%.6g',vals(i)),'HorizontalAlignment','left', ...
        'BackgroundColor',[1 1 1]);
    tx(i).Callback = @(src,~) editApply(src, sl(i));
end

hState = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.82 0.905 0.16 0.050],'String','WAITING', ...
    'FontSize',13,'FontWeight','bold','BackgroundColor',[0.90 0.85 0.55]);
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.82 0.800 0.16 0.075],'String','STOP','FontSize',14, ...
    'FontWeight','bold','BackgroundColor',[0.85 0.35 0.25], ...
    'Callback',@(s,e) setappdata(fig,'stop',true));
hTime = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.82 0.745 0.16 0.042],'String','0:00', ...
    'FontSize',10,'HorizontalAlignment','center');

% Angle and the commanded lean share an axis on purpose: the whole point of
% the outer loop is that the inner one is now chasing a moving target, and
% you cannot judge either trace without the other.
axA = axes(fig,'Position',[0.07 0.505 0.88 0.20]);
hA  = animatedline(axA,'Color',[0.85 0.45 0.05],'LineWidth',1.3);
hR  = animatedline(axA,'Color',[0.20 0.35 0.75],'LineWidth',1.1,'LineStyle','--');
ylabel(axA,'angle [deg]'); grid(axA,'on'); hold(axA,'on');
yline(axA,0,'-','Color',[0.6 0.6 0.6]);
yline(axA,[-1 1]*rad2deg(CAPTURE),':','Color',[0.45 0.55 0.45]);
legend(axA,[hA hR],{'angle','commanded lean'},'Location','northwest', ...
       'Orientation','horizontal','Box','off');

axU = axes(fig,'Position',[0.07 0.295 0.88 0.165]);
hU  = animatedline(axU,'Color',[0.05 0.45 0.50],'LineWidth',1.3);
ylabel(axU,'command u'); grid(axU,'on');
yline(axU,[-U_MAX U_MAX],'--','Color',[0.65 0.25 0.15]);

% The trace that was missing until now. A robot can hold a flawless angle
% while this one ramps off the screen, and that is exactly the failure the
% outer loop exists to fix.
axW = axes(fig,'Position',[0.07 0.065 0.88 0.165]);
hP  = animatedline(axW,'Color',[0.35 0.25 0.55],'LineWidth',1.3);
hV  = animatedline(axW,'Color',[0.75 0.35 0.55],'LineWidth',1.0);
ylabel(axW,'wheel'); xlabel(axW,'t [s]'); grid(axW,'on'); hold(axW,'on');
yline(axW,0,'-','Color',[0.6 0.6 0.6]);
legend(axW,[hP hV],{'travel [rad]','speed [rad/s]'},'Location','northwest', ...
       'Orientation','horizontal','Box','off');

fprintf('\nLay the robot down. Lift it upright to start balancing.\n\n');

% Ten minutes at the telemetry rate, doubled whenever it fills. Preallocating
% for a fixed session length is what forced a time limit in the first place.
N = 36000;
LOG = struct('t',nan(N,1),'angle',nan(N,1),'ref',nan(N,1),'rate',nan(N,1), ...
             'u',nan(N,1),'pos',nan(N,1),'vel',nan(N,1), ...
             'kp',nan(N,1),'ki',nan(N,1),'kd',nan(N,1), ...
             'kv',nan(N,1),'kvi',nan(N,1));

k = 0; nWin = 0; nDraw = 0; balancing = false;
last = [Kp0 Ki0 Kd0]; trimNow = Trim0; shown = vals;
lastOuter = [Kv0 Kvi0 Lean0]; lastYaw = YawD0; lastFric = Ufric0;
t0 = tic; tTx = tic; nFalls = 0;

while true
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
    curOuter = [sl(5).Value sl(6).Value sl(7).Value];
    if any(abs(curOuter - lastOuter) > 1e-4)
        lnk.outer(deg2rad(curOuter(1)), deg2rad(curOuter(2)), deg2rad(curOuter(3)));
        lastOuter = curOuter; tTx = tic;
    end
    if abs(sl(8).Value - lastYaw) > 1e-4
        lnk.yaw(0, sl(8).Value);
        lastYaw = sl(8).Value; tTx = tic;
    end
    if abs(sl(9).Value - lastFric) > 1e-4
        lnk.friction(sl(9).Value, 0.5);
        lastFric = sl(9).Value; tTx = tic;
    end
    % Refresh a box only when its slider actually moved. Rewriting them every
    % pass would wipe out whatever is being typed, one keystroke at a time.
    for i = 1:n
        if abs(sl(i).Value - shown(i)) > 1e-12
            tx(i).String = sprintf('%.6g', sl(i).Value);
            shown(i) = sl(i).Value;
        end
    end

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
            if k > numel(LOG.t)
                LOG = structfun(@(x) [x; nan(numel(x),1)], LOG, ...
                                'UniformOutput', false);
            end
            LOG.t(k)=t; LOG.angle(k)=ang; LOG.ref(k)=d.ref; LOG.rate(k)=d.rate;
            LOG.u(k)=d.u0; LOG.pos(k)=d.pos_fwd; LOG.vel(k)=d.vel_fwd;
            LOG.kp(k)=last(1); LOG.ki(k)=last(2); LOG.kd(k)=last(3);
            LOG.kv(k)=lastOuter(1); LOG.kvi(k)=lastOuter(2);
            addpoints(hU, t, d.u0);
            addpoints(hA, t, rad2deg(ang));
            addpoints(hR, t, rad2deg(d.ref));
            addpoints(hP, t, d.pos_fwd);
            addpoints(hV, t, d.vel_fwd);
        end
    end

    nDraw = nDraw + 1;
    if mod(nDraw, 10) == 0
        xlim(axA,[max(0,t-12) max(12,t)]); xlim(axU,[max(0,t-12) max(12,t)]);
        xlim(axW,[max(0,t-12) max(12,t)]);
        hTime.String = sprintf('%d:%02d   %d samples', ...
                               floor(t/60), floor(mod(t,60)), k);
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
fprintf('  final inner gains   : Kp=%.2f  Ki=%.2f  Kd=%.2f  trim=%.2f deg\n', ...
        last, rad2deg(trimNow));
fprintf('  final outer gains   : Kv=%.3f  Kvi=%.3f  lean limit=%.1f deg\n', ...
        lastOuter(1), lastOuter(2), lastOuter(3));
fprintf('  angle RMS           : %.2f deg\n', rad2deg(rms(T.angle)));
fprintf('  max |angle|         : %.2f deg\n', rad2deg(max(abs(T.angle))));
fprintf('  command RMS         : %.3f  (saturated %.1f%%)\n', ...
        rms(T.u), 100*mean(abs(T.u) >= U_MAX*0.99));

% --- how the wheel loop did ---------------------------------------------
leanLim = deg2rad(lastOuter(3));
fprintf('  wheel drift         : %.2f rad  (max |travel| %.2f rad)\n', ...
        T.pos(end)-T.pos(1), max(abs(T.pos)));
fprintf('  wheel speed RMS     : %.2f rad/s\n', rms(T.vel));
if leanLim > 0
    fprintf('  lean commanded RMS  : %.2f deg  (clamped %.1f%%%% of the time)\n', ...
            rad2deg(rms(T.ref)), 100*mean(abs(T.ref) >= leanLim*0.99));
else
    fprintf('  lean commanded RMS  : outer loop disabled (Lean = 0)\n');
end

% Sign check, measured rather than assumed. Leaning one way must make the
% robot accelerate the other way; if it does not, Kv is pushing the runaway
% instead of arresting it and the slider has to go negative. Restricted to
% contiguous samples, because the derivative across a fall is meaningless.
ok = dt > 0 & dt < 0.2;
if nnz(ok) > 50
    dv = diff(T.vel) ./ dt;
    a  = T.angle(1:end-1);
    c  = corrNoToolbox(a(ok), dv(ok));
    if c < -0.05
        fprintf('  Kv sign             : positive is correct (corr %.2f)\n', c);
    elseif c > 0.05
        fprintf(2, '  Kv sign             : FLIP IT NEGATIVE (corr %+.2f)\n', c);
        fprintf(2, '    Leaning one way accelerates the robot the same way here,\n');
        fprintf(2, '    so a positive Kv feeds the runaway instead of stopping it.\n');
    else
        fprintf('  Kv sign             : inconclusive (corr %+.2f), needs a longer run\n', c);
    end
end

if ~exist('../logs','dir'), mkdir('../logs'); end
fname = sprintf('../logs/s02_balance_%s.mat', ...
                string(datetime('now','Format','yyyyMMdd_HHmmss')));
save(fname,'T','U_MAX','last','lastOuter','trimNow','nFalls');
fprintf('  saved               : %s\n', fname);

fprintf(['\nTuning order — inner loop first, with Kv and Kvi at 0:\n' ...
         '  1. Ki=0, Kd=0. Raise Kp until it reacts briskly and starts to\n' ...
         '     oscillate fast. Note that value.\n' ...
         '  2. Drop Kp to ~60%% of it, raise Kd until the oscillation goes.\n' ...
         '  3. Nudge Trim until the wheel travel trace stops ramping one way.\n' ...
         '  4. Add Ki (0.5..2) last, only to remove steady-state lean.\n' ...
         '\nThen the outer loop, which is what keeps it on the table:\n' ...
         '  5. Raise Kv until the robot stops accelerating away and settles\n' ...
         '     to a slow shuffle. Watch the wheel SPEED trace, not the angle.\n' ...
         '  6. Add Kvi until the travel trace returns towards zero instead of\n' ...
         '     just holding still wherever it stopped. Too much makes it\n' ...
         '     surge slowly back and forth.\n' ...
         '  7. If "clamped %%" above is high, the lean limit is the bottleneck:\n' ...
         '     raise Lean a degree at a time. If it is near zero, do not.\n' ...
         '  8. YawD only if it wanders left-right. Ufric only if it hunts in\n' ...
         '     place at a good tune - that symptom is stiction, not gains.\n']);

function editApply(src, slh)
%EDITAPPLY  Commit a typed gain to its slider.
%   A number outside the slider's range widens the slider instead of being
%   clamped away: at this stage the typed value is the intent, and silently
%   ignoring it is the one behaviour that would cost a tuning session.
%   Anything unparseable reverts to the slider's current value.
    v = str2double(src.String);
    if isnan(v) || ~isscalar(v)
        src.String = sprintf('%.6g', slh.Value);
        return
    end
    if v > slh.Max, slh.Max = v; end
    if v < slh.Min, slh.Min = v; end
    slh.Value  = v;
    src.String = sprintf('%.6g', v);
end

function s = maxRun(t)
    gaps = [0; find(diff(t) >= 0.2); numel(t)];
    s = 0;
    for i = 1:numel(gaps)-1
        seg = t(gaps(i)+1 : gaps(i+1));
        if numel(seg) > 1, s = max(s, seg(end) - seg(1)); end
    end
end

function c = corrNoToolbox(a, b)
%CORRNOTOOLBOX  Pearson correlation without the Statistics Toolbox, which
%   nothing else in this project needs either.
    a = a(:) - mean(a);
    b = b(:) - mean(b);
    den = sqrt(sum(a.^2) * sum(b.^2));
    if den == 0, c = 0; else, c = sum(a.*b) / den; end
end
