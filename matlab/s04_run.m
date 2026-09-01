%% s04_run — run the robot on one fixed set of gains, and measure it.
%  s02 is for hunting a tune, s04 is for living with one. Edit the block
%  below, run, stand the robot up. It arms and disarms itself, so a fall is
%  not the end of the run. Each unbroken stretch is scored separately, since
%  travel measured across a fall is the fall, not the travel.
%
%  Ends on STOP, 'q', closing the window, or Ctrl+C. The numbers come from
%  sbrMetrics, the same file s03 uses, so two runs compare directly.

clear; clc; close all;

% ======================================================================
%  THE ONLY BLOCK YOU EDIT
% ======================================================================
PORT = "192.168.4.1";          % or "COM4" for USB

% Inner loop, 200 Hz: holds the tilt angle. Yours, from s02.
Kp   = 8.556;
Ki   = 11.988;
Kd   = 0.393;
Trim = 0.0;                    % deg, where "upright" actually is

% Outer loop, 40 Hz: picks the tilt the inner loop is told to hold.
% From the s03 run of 2026-09-01. SbrLink converts these to radians.
Kv    = 4.72;                  % deg of lean per (rad/s) of wheel speed
Kvi   = 1.335;                 % deg of lean per rad of wheel travel
Lean  = 3.914;                 % deg, the outer loop's whole authority
YawD  = 0.367;                 % heading damping
Ufric = 0.08691;               % V, Coulomb feedforward. 0 disables it.

% Your original outer loop. Swap the comments to run it instead, then compare
% the two "median over" blocks - the s03 play-off could not settle this.
% Kv    = 3.0;
% Kvi   = 1.0;
% Lean  = 3.52;
% Ufric = 0.0;

U_MAX    = 3.0;
TILT_MAX = deg2rad(30);
% ======================================================================

WHEEL_R   = 0.045;             % m, for reporting travel in cm
CAPTURE   = deg2rad(10);
CAPTURE_N = 5;
MIN_SEG   = 5.0;               % s. Shorter stretches are not worth scoring.

% ---------------------------------------------------------------- link
lnk = SbrLink(PORT);
cleanupObj = onCleanup(@() delete(lnk));

d = []; tw = tic;
while toc(tw) < 1.5
    dn = lnk.readLatest();
    if ~isempty(dn), d = dn; end
end
assert(~isempty(d), 'No telemetry on %s. Run s00_motor_check first.', PORT);
assert(d.foc_hz >= 0, 'initFOC() failed on the motor node. See s00_motor_check.');
assert(d.imu_ok, 'imu_ok = 0. The IMU node is not delivering attitude.');
fprintf('link ok: foc %.0f Hz, imu %.0f Hz\n', d.foc_hz, d.imu_hz);

trimRad = deg2rad(Trim);
lnk.limits(U_MAX, TILT_MAX);
lnk.trim(trimRad);
lnk.setpoint(0);
lnk.gains(Kp, Ki, Kd);
lnk.outer(deg2rad(Kv), deg2rad(Kvi), deg2rad(Lean));
lnk.yaw(0, YawD);
lnk.friction(Ufric, 0.5);
lnk.mode(2);

fprintf('\ngains on the robot:\n');
fprintf('  inner   Kp %.4g   Ki %.4g   Kd %.4g   Trim %.4g deg\n', Kp, Ki, Kd, Trim);
fprintf('  outer   Kv %.4g   Kvi %.4g   Lean %.4g deg   Ufric %.4g V\n', ...
        Kv, Kvi, Lean, Ufric);
fprintf('\nstand it up.\n');

% ---------------------------------------------------------------- window
fig = figure('Name','s04 — running on fixed gains (Q = stop)', ...
             'NumberTitle','off','Position',[100 60 1120 780]);
setappdata(fig,'stop',false);
fig.KeyPressFcn = @(s,e) setappdata(s,'stop', strcmpi(e.Key,'q'));

uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.86 0.945 0.10 0.042],'String','STOP','FontSize',12, ...
    'FontWeight','bold','BackgroundColor',[0.85 0.35 0.25], ...
    'Callback',@(s,e) setappdata(fig,'stop',true));

hState = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.07 0.945 0.14 0.038],'String','WAITING', ...
    'FontSize',12,'FontWeight','bold','BackgroundColor',[0.90 0.85 0.55]);

hTime = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.23 0.945 0.60 0.038],'String','', ...
    'FontSize',11,'HorizontalAlignment','left');

% Angle and commanded lean share an axis: the inner loop chases a moving
% target, and neither trace means anything without the other.
axA = axes(fig,'Position',[0.07 0.655 0.88 0.255]);
hA  = animatedline(axA,'Color',[0.85 0.45 0.05],'LineWidth',1.3);
hR  = animatedline(axA,'Color',[0.20 0.35 0.75],'LineWidth',1.1,'LineStyle','--');
ylabel(axA,'angle [deg]'); grid(axA,'on'); hold(axA,'on');
yline(axA,0,'-','Color',[0.6 0.6 0.6]);
yline(axA,[-1 1]*Lean,':','Color',[0.35 0.45 0.70]);
legend(axA,[hA hR],{'angle','commanded lean'},'Location','northwest', ...
       'Orientation','horizontal','Box','off');

axU = axes(fig,'Position',[0.07 0.375 0.88 0.215]);
hU  = animatedline(axU,'Color',[0.05 0.45 0.50],'LineWidth',1.3);
ylabel(axU,'command u [V]'); grid(axU,'on');
yline(axU,[-U_MAX U_MAX],'--','Color',[0.65 0.25 0.15]);

% A robot can hold a flawless angle while this trace walks off the bench.
axW = axes(fig,'Position',[0.07 0.075 0.88 0.235]);
hP  = animatedline(axW,'Color',[0.35 0.25 0.55],'LineWidth',1.3);
hV  = animatedline(axW,'Color',[0.75 0.35 0.55],'LineWidth',1.0);
ylabel(axW,'wheel'); xlabel(axW,'t [s]'); grid(axW,'on'); hold(axW,'on');
yline(axW,0,'-','Color',[0.6 0.6 0.6]);
legend(axW,[hP hV],{'travel [rad]','speed [rad/s]'},'Location','northwest', ...
       'Orientation','horizontal','Box','off');

% ---------------------------------------------------------------- run
N = 40000;
LOG = struct('t',nan(N,1),'seg',nan(N,1),'tseg',nan(N,1),'angle',nan(N,1), ...
             'ref',nan(N,1),'rate',nan(N,1),'u',nan(N,1),'pos',nan(N,1), ...
             'vel',nan(N,1));
k = 0; nDraw = 0; nWin = 0; nFalls = 0; seg = 0; tSeg = 0;
balancing = false;
t0 = tic; tTx = tic;

while ~getappdata(fig,'stop') && isvalid(fig)
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
    ang = d.angle - trimRad;

    if ~balancing
        if abs(ang) < CAPTURE
            nWin = nWin + 1;
            if nWin >= CAPTURE_N
                lnk.arm(true);
                balancing = true; nWin = 0;
                seg = seg + 1; tSeg = t;
                hState.String = 'BALANCING';
                hState.BackgroundColor = [0.55 0.80 0.55];
                fprintf('  stretch %d starts at t = %5.1f s\n', seg, t);
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
            fprintf('  lost it at t = %5.1f s after %.1f s (%.0f deg) — stand it up\n', ...
                    t, t - tSeg, rad2deg(ang));
        else
            k = k + 1;
            if k > numel(LOG.t)
                LOG = structfun(@(x) [x; nan(numel(x),1)], LOG, ...
                                'UniformOutput', false);
            end
            LOG.t(k)=t;        LOG.seg(k)=seg;      LOG.tseg(k)=t - tSeg;
            LOG.angle(k)=ang;  LOG.ref(k)=d.ref;    LOG.rate(k)=d.rate;
            LOG.u(k)=d.u0;     LOG.pos(k)=d.pos_fwd; LOG.vel(k)=d.vel_fwd;
            addpoints(hA, t, rad2deg(ang));
            addpoints(hR, t, rad2deg(d.ref));
            addpoints(hU, t, d.u0);
            addpoints(hP, t, d.pos_fwd);
            addpoints(hV, t, d.vel_fwd);
        end
    end

    nDraw = nDraw + 1;
    if mod(nDraw, 10) == 0
        for ax = [axA axU axW]
            xlim(ax, [max(0,t-15) max(15,t)]);
        end
        if balancing
            hTime.String = sprintf('%d:%02d   up %.0f s   stretch %d   %d falls', ...
                floor(t/60), floor(mod(t,60)), t - tSeg, seg, nFalls);
        else
            hTime.String = sprintf('%d:%02d   waiting   %d falls', ...
                floor(t/60), floor(mod(t,60)), nFalls);
        end
        drawnow limitrate;
    end
end

lnk.arm(false);
lnk.stop();

% ---------------------------------------------------------------- report
v = ~isnan(LOG.t);
T = struct2table(structfun(@(x) x(v), LOG, 'UniformOutput', false));

fprintf('\n=== Result ===\n');
fprintf('  inner   Kp %.4g   Ki %.4g   Kd %.4g   Trim %.4g deg\n', Kp, Ki, Kd, Trim);
fprintf('  outer   Kv %.4g   Kvi %.4g   Lean %.4g deg   Ufric %.4g V\n\n', ...
        Kv, Kvi, Lean, Ufric);

if isempty(T)
    fprintf('  It never balanced long enough to record anything.\n');
    return
end

segs = unique(T.seg)';
M = []; durs = [];
fprintf('  %-4s %7s %9s %7s %8s %8s %9s %9s\n', ...
        'run', 'up [s]', 'travel', 'sway', 'wheel', 'buzz', 'chatter', 'wobble');
for s = segs
    idx = T.seg == s;
    dur = T.tseg(find(idx,1,'last'));
    durs = [durs dur]; %#ok<AGROW>
    if dur < MIN_SEG || sum(idx) < 40
        fprintf('  %-4d %7.1f   too short to score\n', s, dur);
        continue
    end
    X = [T.tseg(idx) T.angle(idx) T.rate(idx) T.vel(idx) T.pos(idx) T.u(idx)];
    m = sbrMetrics(X, 'ok', dur);
    M = [M m]; %#ok<AGROW>
    fprintf('  %-4d %7.1f %9.1f %7.1f %8.2f %8.3f %9.1f %9.1f\n', s, dur, ...
        100*WHEEL_R*m.pos, 100*WHEEL_R*m.sway, m.vel, ...
        rad2deg(m.angHf), m.jerk, rad2deg(m.angP2P));
end
fprintf('  %-4s %7s %9s %7s %8s %8s %9s %9s\n', ...
        '', '', 'cm p-p', 'cm', 'rad/s', 'deg', 'V/s', 'deg p-p');

if ~isempty(M)
    med = @(f) median(arrayfun(@(x) x.(f), M));
    fprintf('\n  median over %d scored run(s):\n', numel(M));
    fprintf('    travel  %6.1f cm p-p        the course it walked\n', 100*WHEEL_R*med('pos'));
    fprintf('    sway    %6.1f cm            fore-aft rocking, 0.2-1.5 Hz\n', 100*WHEEL_R*med('sway'));
    fprintf('    wheel   %6.2f rad/s         shuffling\n', med('vel'));
    fprintf('    buzz    %6.3f deg           tilt energy above 0.8 Hz\n', rad2deg(med('angHf')));
    fprintf('    chatter %6.1f V/s           how hard the motors are working\n', med('jerk'));
    fprintf('    wobble  %6.1f deg p-p       the visible swing\n', rad2deg(med('angP2P')));
end

fprintf('\n  stood up %d time(s), %d fall(s)\n', numel(segs), nFalls);
if ~isempty(durs)
    fprintf('  longest unbroken run %.1f s, total balancing %.1f s\n', ...
            max(durs), sum(durs));
end

outDir = fullfile(fileparts(mfilename('fullpath')), '..', 'logs');
if ~exist(outDir,'dir'), mkdir(outDir); end
fname = fullfile(outDir, sprintf('s04_run_%s.mat', ...
                string(datetime('now','Format','yyyyMMdd_HHmmss'))));
gains = struct('Kp',Kp,'Ki',Ki,'Kd',Kd,'Trim',Trim, ...
               'Kv',Kv,'Kvi',Kvi,'Lean',Lean,'YawD',YawD,'Ufric',Ufric);
save(fname,'T','gains','M','durs','nFalls','U_MAX','WHEEL_R');
fprintf('\n  saved: %s\n', fname);
