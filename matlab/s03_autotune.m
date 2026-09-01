function s03_autotune(simulate, budget)
%S03_AUTOTUNE  Tunes the wheel loop to hold station, live, while it balances.
%
%   s03_autotune              tune the real robot
%   s03_autotune(true)        rehearse against a model, no robot involved
%   s03_autotune(true, 12)    short rehearsal
%
%   Stand the robot up and leave it alone. Your inner PID is held fixed; the
%   outer loop is searched. Every ~20 s it tries a set, scores it, keeps it or
%   backs off. You are only needed when it falls.
%
%   Lean is searched, not fixed: in the s02 logs the position term sat clamped
%   68-100% of the time, so travel was fed back as a constant shove rather
%   than a proportional correction. No value of Kvi fixes that, only more
%   authority does.
%
%   If the result buzzes, raise W.angHf and W.jerk. If it still tours the
%   bench, raise W.pos. If it rocks fore-aft, raise W.sway.

arguments
    simulate (1,1) logical = false
    budget   (1,1) double  = 36
end

% ---------------------------------------------------------------- settings
PORT = "192.168.4.1";          % or "COM4" for USB
WHEEL_R = 0.045;               % m, for reporting travel in cm

SEED = struct('Kp', 8.556, 'Ki', 11.988, 'Kd', 0.393, ...
              'Kv', 3.0,   'Kvi', 1.0,   'Lean', 3.52, 'Ufric', 0.0);

TUNE_INNER = false;            % true also searches Kp/Ki/Kd

% name, hard bounds the tuner may never leave, initial trust half-width.
SPEC = { 'Kv',    [ 0.5 14.0], 2.50
         'Kvi',   [ 0.2 12.0], 2.00
         'Lean',  [ 2.0  6.0], 1.20
         'Ufric', [ 0.0  0.6], 0.30 };
if TUNE_INNER
    SPEC = [SPEC
            'Kp',  [ 3.0 25.0], 2.00
            'Ki',  [ 0.0 20.0], 4.00
            'Kd',  [0.05  3.0], 0.35 ];
end

YAWD  = 0.367;
TRIM  = 0.0;                   % deg

U_MAX     = 3.0;
TILT_MAX  = deg2rad(30);
CAPTURE   = deg2rad(10);
CAPTURE_N = 5;

STAGES  = 3;
SETTLE  = 2.5;                 % s discarded after a gain change
RECORD  = 18.0;                % s scored; a slow walk needs time to show
CONFIRM = 3;                   % repeats in the final play-off

% Cut an episode past any of these. The multiples adapt to your baseline, the
% caps stop a poor baseline licensing a worse one, and the floor stops a cap
% tighter than the baseline from aborting the baseline itself.
ABORT_MULT  = 2.2;             % x baseline tilt swing
ABORT_MAX   = deg2rad(12);     % ceiling on that allowance
ABORT_VMULT = 3.0;             % x baseline wheel speed
ABORT_VMAX  = 4.0;             % rad/s ceiling
ABORT_PMULT = 1.6;             % x baseline travel
ABORT_PMAX  = 7.0;             % rad ceiling, ~32 cm
ABORT_FLOOR = 1.4;             % never tighter than this x baseline
GROW = 1.35; SHRINK = 0.65;    % trust region on success / on failure

% Weights on the metrics in sbrMetrics. angP2P and rate are cheap on purpose:
% holding a spot requires leaning, so charging hard for tilt swing while
% asking for a short course makes the tuner give up on the course. angHf and
% jerk are what forbid the twitchy solution instead.
W = struct('pos',3.0, 'sway',2.0, 'vel',1.0, ...
           'angHf',1.5, 'jerk',1.0, 'angP2P',0.5, 'rate',0.5);

names = SPEC(:,1)';
hard  = SPEC(:,2);
w0    = cell2mat(SPEC(:,3))';

cfg = struct('trim',deg2rad(TRIM),'uMax',U_MAX, ...
             'tiltMax',TILT_MAX,'capture',CAPTURE,'captureN',CAPTURE_N, ...
             'settle',SETTLE,'record',RECORD,'sim',simulate, ...
             'wheelR',WHEEL_R,'W',W);

% ---------------------------------------------------------------- link
lnk = [];
if simulate
    fprintf('SIMULATION - no robot. Rehearsing the tuner only.\n\n');
else
    lnk = SbrLink(PORT);
    cleanupLink = onCleanup(@() safeShutdown(lnk));
    d = []; tw = tic;
    while toc(tw) < 1.5
        dn = lnk.readLatest();
        if ~isempty(dn), d = dn; end
    end
    assert(~isempty(d), 'No telemetry on %s. Run s00_motor_check first.', PORT);
    assert(d.foc_hz >= 0, 'initFOC() failed. See s00_motor_check.');
    assert(d.imu_ok, 'imu_ok = 0. The IMU node is not delivering attitude.');
    fprintf('link ok: foc %.0f Hz, imu %.0f Hz\n', d.foc_hz, d.imu_hz);
    lnk.limits(U_MAX, TILT_MAX);
    lnk.trim(cfg.trim);
    lnk.setpoint(0);
    lnk.yaw(0, YAWD);
    lnk.mode(2);
end

con = makeConsole(WHEEL_R);
cleanupFig = onCleanup(@() closeIfValid(con));

% ---- state shared with the nested objective ------------------------------
standing  = false;             % is the robot still up between episodes
incumbent = SEED;              % gains to fall back to after an abort
ref       = [];                % baseline metrics, the scoring reference
abortAt   = struct('p2p',inf,'vel',inf,'pos',inf);
nAbort = 0; nFall = 0; nRun = 0;

% ---------------------------------------------------------------- baseline
fprintf('\n=== Baseline: your gains, %d runs ===\n', CONFIRM);
base = [];
for i = 1:CONFIRM
    m = runEpisode(SEED);
    fprintf('  run %d: %s\n', i, describe(m, WHEEL_R));
    if ~m.aborted, base = [base m]; end %#ok<AGROW>
    if stopped(con), fprintf('Stopped.\n'); return; end
end
assert(~isempty(base), 'The seed gains never completed a run. Fix them in s02 first.');
ref = medianMetrics(base);
abortAt = struct( ...
    'p2p', max(ABORT_FLOOR*ref.angP2P, min(ABORT_MULT*ref.angP2P,  ABORT_MAX)), ...
    'vel', max(ABORT_FLOOR*ref.vel,    min(ABORT_VMULT*ref.vel,    ABORT_VMAX)), ...
    'pos', max(ABORT_FLOOR*ref.pos,    min(ABORT_PMULT*ref.pos,    ABORT_PMAX)));
setBaseline(con, ref.pos);
fprintf('  reference : %s\n', describe(ref, WHEEL_R));
fprintf('  cut an episode at %.2f deg swing, %.1f rad/s wheel, or %.0f cm travel\n', ...
        rad2deg(abortAt.p2p), abortAt.vel, 100*WHEEL_R*abortAt.pos);

% ---------------------------------------------------------------- stages
perStage = max(6, floor(budget/STAGES));
w = w0;
incScore = 1.0;
carryX = []; carryY = []; carryC = [];

for stage = 1:STAGES
    if stopped(con), break; end
    vars = [];
    for i = 1:numel(names)
        c  = incumbent.(names{i});
        lo = max(hard{i}(1), c - w(i));
        hi = min(hard{i}(2), c + w(i));
        if hi - lo < 1e-6, hi = lo + 1e-6; end
        vars = [vars optimizableVariable(names{i}, [lo hi])]; %#ok<AGROW>
    end
    fprintf('\n=== Stage %d of %d, %d episodes ===\n', stage, STAGES, perStage);
    for i = 1:numel(names)
        fprintf('  %-6s %7.3f  [%7.3f %7.3f]\n', names{i}, ...
                incumbent.(names{i}), vars(i).Range(1), vars(i).Range(2));
    end

    [X0, Y0, C0] = seedPoints(incumbent, names, vars, carryX, carryY, carryC);
    % bayesopt charges carried-over points to MaxObjectiveEvaluations without
    % re-running them, so budget the new episodes - those are what cost time.
    nSeeded = sum(~isnan(Y0));
    args = {'MaxObjectiveEvaluations', perStage + nSeeded, ...
            'IsObjectiveDeterministic', false, ...
            'NumCoupledConstraints', 1, ...
            'AcquisitionFunctionName', 'expected-improvement-plus', ...
            'InitialX', X0, ...
            'PlotFcn', {}, 'Verbose', 0, ...
            'OutputFcn', {@(a,b) stopped(con)}};
    if ~isempty(Y0)
        args = [args, {'InitialObjective', Y0, ...
                       'InitialConstraintViolations', C0}]; %#ok<AGROW>
    end
    r = bayesopt(@objective, vars, args{:});

    carryX = r.XTrace; carryY = r.ObjectiveTrace; carryC = r.ConstraintsTrace;

    % An infeasible stage is an outcome, not an error: everything in the trust
    % region got cut short. Shrink and try again.
    cand = incumbent; candScore = inf;
    try
        b = table2struct(bestPoint(r, 'Criterion', 'min-visited-mean'));
        for i = 1:numel(names), cand.(names{i}) = b.(names{i}); end
        candScore = r.MinEstimatedObjective;
    catch
        fprintf('  every episode in this region was cut short.\n');
    end
    if ~isfinite(candScore), candScore = inf; end

    if candScore < incScore * 0.98
        incumbent = cand;
        incScore  = candScore;
        w = min(w * GROW, w0 * 1.5);
        fprintf('  improved to %.3f. Trust region widened.\n', incScore);
    else
        w = w * SHRINK;
        fprintf('  no improvement (%.3f vs %.3f). Trust region narrowed.\n', ...
                candScore, incScore);
    end
end

% ---------------------------------------------------------------- play-off
fprintf('\n=== Play-off: yours vs the tuner, %d runs each ===\n', CONFIRM);
mA = []; mD = [];
for i = 1:CONFIRM
    m = runEpisode(incumbent);
    fprintf('  tuner %d: %s\n', i, describe(m, WHEEL_R));
    mA = [mA m]; %#ok<AGROW>
    if stopped(con), break; end
end
for i = 1:CONFIRM
    m = runEpisode(SEED);
    fprintf('  yours %d: %s\n', i, describe(m, WHEEL_R));
    mD = [mD m]; %#ok<AGROW>
    if stopped(con), break; end
end
if ~simulate, lnk.arm(false); end

sA = arrayfun(@(x) scoreOf(x, ref, W), mA);
sD = arrayfun(@(x) scoreOf(x, ref, W), mD);

% ---------------------------------------------------------------- report
fprintf('\n=== Result ===\n');
fprintf('  %-6s %10s %10s %9s\n', 'gain', 'yours', 'tuner', 'change');
for i = 1:numel(names)
    nm = names{i};
    if SEED.(nm) == 0
        ch = '      new';
    else
        ch = sprintf('%8.0f%%', 100*(incumbent.(nm)/SEED.(nm) - 1));
    end
    fprintf('  %-6s %10.4g %10.4g %s\n', nm, SEED.(nm), incumbent.(nm), ch);
end

mAm = medianMetrics(mA); mDm = medianMetrics(mD);
fprintf('\n  %-8s %10s %10s %9s\n', 'metric', 'yours', 'tuner', 'change');
showMetric('travel',  100*WHEEL_R*mDm.pos,  100*WHEEL_R*mAm.pos,  'cm p-p');
showMetric('sway',    100*WHEEL_R*mDm.sway, 100*WHEEL_R*mAm.sway, 'cm rocking');
showMetric('wheel',   mDm.vel,              mAm.vel,              'rad/s');
showMetric('buzz',    rad2deg(mDm.angHf),   rad2deg(mAm.angHf),   'deg >0.8Hz');
showMetric('chatter', mDm.jerk,             mAm.jerk,             'V/s');
showMetric('wobble',  rad2deg(mDm.angP2P),  rad2deg(mAm.angP2P),  'deg p-p');

fprintf('\n  score, yours : %.3f\n', median(sD));
fprintf('  score, tuner : %.3f\n', median(sA));
fprintf('  episodes: %d run, %d cut short, %d falls\n', nRun, nAbort, nFall);

if median(sA) < median(sD) && max(sA) < median(sD)
    fprintf('\n  The tuner wins by %.0f%%, and its worst run beat your median.\n', ...
            100*(1 - median(sA)/median(sD)));
    fprintf('  Paste into the boxes in s02_balance:\n    ');
    for i = 1:numel(names)
        fprintf('%s %.4g   ', names{i}, incumbent.(names{i}));
    end
    fprintf('\n');
elseif median(sA) < median(sD)
    fprintf('\n  Better on the median but the runs overlap. Re-run with a\n');
    fprintf('  larger CONFIRM before trusting it.\n');
else
    fprintf('\n  No improvement that survives repetition. If it still tours the\n');
    fprintf('  bench, the position term is probably still saturated - check the\n');
    fprintf('  Lean it settled on against the 6 deg cap in SPEC.\n');
end

outDir = fullfile(fileparts(mfilename('fullpath')), '..', 'logs');
if ~exist(outDir,'dir'), mkdir(outDir); end
fname = fullfile(outDir, sprintf('s03_autotune_%s.mat', ...
                string(datetime('now','Format','yyyyMMdd_HHmmss'))));
tried = carryX; scores = carryY;
save(fname,'tried','scores','SEED','incumbent','ref','sA','sD','cfg','W');
fprintf('\n  saved: %s\n', fname);

% =====================================================================
% Nested - these share standing / incumbent / ref / counters.
% =====================================================================
    function [J, c] = objective(tbl)
        p = incumbent;
        for q = 1:numel(names), p.(names{q}) = tbl.(names{q}); end
        m = runEpisode(p);
        J = scoreOf(m, ref, W);
        % A cut episode is a constraint violation, not an invented objective
        % value: bayesopt models it separately and steers around the region.
        c = 1;
        if ~m.aborted, c = -1; end
    end

    function m = runEpisode(p)
        nRun = nRun + 1;
        if cfg.sim
            [X, why] = simRun(p, cfg, abortAt);
        else
            [X, why] = realRun(lnk, p, cfg, abortAt, standing);
            standing = strcmp(why, 'ok');
            if ~standing
                % Known-good gains back on before the operator touches it.
                lnk.gains(incumbent.Kp, incumbent.Ki, incumbent.Kd);
                lnk.outer(deg2rad(incumbent.Kv), deg2rad(incumbent.Kvi), ...
                          deg2rad(incumbent.Lean));
                lnk.friction(incumbent.Ufric, 0.5);
            end
        end
        switch why
            case 'fell',   nFall  = nFall  + 1;
            case 'wobble', nAbort = nAbort + 1;
        end
        m = sbrMetrics(X, why, cfg.record);
        noteEpisode(con, m);
    end
end

% =========================================================================
function [X, why] = realRun(lnk, p, cfg, abortAt, standing)
%REALRUN  One episode. Gains change live; re-armed only if it is down.
lnk.gains(p.Kp, p.Ki, p.Kd);
lnk.outer(deg2rad(p.Kv), deg2rad(p.Kvi), deg2rad(p.Lean));
lnk.friction(p.Ufric, 0.5);

X = zeros(0,6);
if ~standing
    if ~waitUpright(lnk, cfg), why = 'fell'; return; end
    lnk.arm(true);
end

open = struct('p2p',inf,'vel',inf,'pos',inf);
[~, why] = pump(lnk, cfg, cfg.settle, false, open);
if ~strcmp(why,'ok'), return; end
[X, why] = pump(lnk, cfg, cfg.record, true, abortAt);
end

% -------------------------------------------------------------------------
function ok = waitUpright(lnk, cfg)
ok = false; nWin = 0; asked = false; t = tic;
while toc(t) < 90
    lnk.heartbeat();
    d = lnk.readLatest();
    if isempty(d), pause(0.01); continue; end
    if abs(d.angle - cfg.trim) < cfg.capture
        nWin = nWin + 1;
        if nWin >= cfg.captureN, ok = true; return; end
    else
        nWin = 0;
        if ~asked, fprintf('    (stand it up)\n'); asked = true; end
    end
    pause(0.005);
end
end

% -------------------------------------------------------------------------
function [X, why] = pump(lnk, cfg, secs, collect, abortAt)
%PUMP  Feed the watchdog, read telemetry, watch for trouble.
%   Swing and speed are judged on a rolling 2 s window so a single knock does
%   not end an episode; travel is judged outright, being already slow.
X = zeros(0,6); why = 'ok';
t = tic; tTx = tic; tChk = tic; k = 0;
if collect, X = nan(ceil(secs*80), 6); end
while toc(t) < secs
    if toc(tTx) > 0.1, lnk.heartbeat(); tTx = tic; end
    d = lnk.readLatest();
    if isempty(d), pause(0.002); continue; end
    if d.tilt_fault || ~d.armed, why = 'fell'; break; end
    if collect
        k = k + 1;
        if k > size(X,1), X = [X; nan(size(X,1),6)]; end %#ok<AGROW>
        X(k,:) = [toc(t), d.angle - cfg.trim, d.rate, d.vel_fwd, d.pos_fwd, d.u0];
        if isfinite(abortAt.pos) && abs(d.pos_fwd) > abortAt.pos
            why = 'wobble'; break
        end
        if toc(tChk) > 0.5 && isfinite(abortAt.p2p)
            tChk = tic;
            win = X(max(1,k-160):k, [2 4]);
            if size(win,1) > 40 && tooFar(win, abortAt)
                why = 'wobble'; break
            end
        end
    end
    pause(0.002);
end
if collect, X = X(1:k, :); end
end

% -------------------------------------------------------------------------
function [X, why] = simRun(p, cfg, abortAt)
%SIMRUN  Linearised wheeled inverted pendulum, discretised as the firmware
%   does it, with Coulomb friction so Ufric has something to compensate.
%   For rehearsing the tuner, not for finding gains.
g = 9.81; l = 0.10; r = 0.045; c = 2.0; bias = deg2rad(0.4);
uStick = 0.20;                                  % motor stiction, volts
CTRL_HZ = 200; OUTER_DIV = 5; VEL_LPF_HZ = 2.0;
dt = 1/CTRL_HZ; dto = OUTER_DIV/CTRL_HZ;

kv = deg2rad(p.Kv); kvi = deg2rad(p.Kvi); leanLim = deg2rad(p.Lean);
th = deg2rad(1.5); thd = 0; pp = 0; pv = 0;
integ = 0; refOut = 0; vf = 0; od = 0;
a = 1 - exp(-2*pi*VEL_LPF_HZ*dto);

n = round((cfg.settle + cfg.record)/dt);
nSkip = round(cfg.settle/dt);
X = nan(n-nSkip, 6); k = 0; why = 'ok';

for i = 1:n
    od = od + 1;
    if od >= OUTER_DIV
        od = 0;
        vf = vf + a*(-pv/r - vf);
        half = 0.5*leanLim;
        refOut = clip(clip(kv*vf, leanLim) + clip(kvi*(-pp/r), half), leanLim);
    end
    err   = refOut - (th + bias) + 0.0004*randn;
    integ = clip(integ + p.Ki*err*dt, cfg.uMax);
    u     = clip(p.Kp*err + integ - p.Kd*thd, cfg.uMax);
    uff   = p.Ufric * clip((-pv/r)/0.5, 1);     % the same ramp as fricFF()
    ua    = clip(u + uff, cfg.uMax);

    % The wheel does not move until the command beats stiction, which is what
    % makes a torque-mode gimbal drive limit-cycle in place.
    drive = ua - uStick*clip((-pv/r)/0.05, 1);
    pa  = -c*drive + 0.02*randn;
    tha = (g/l)*th - pa/l;
    thd = thd + tha*dt;  th = th + thd*dt;
    pv  = pv  + pa*dt;   pp = pp + pv*dt;

    if abs(th) > cfg.tiltMax, why = 'fell'; break; end
    if i > nSkip
        k = k + 1;
        X(k,:) = [(i-nSkip)*dt, th, thd, -pv/r, -pp/r, ua];
        if isfinite(abortAt.pos) && abs(X(k,5)) > abortAt.pos
            why = 'wobble'; break
        end
        if mod(k,25) == 0 && isfinite(abortAt.p2p) && k > 40
            if tooFar(X(max(1,k-160):k, [2 4]), abortAt), why = 'wobble'; break; end
        end
    end
end
X = X(1:max(k,0), :);
end

% -------------------------------------------------------------------------
function J = scoreOf(m, ref, W)
%SCOREOF  Relative to the baseline: 1.00 is exactly your current tune. A cut
%   episode still scores on the data it produced, plus a penalty for ending
%   early, so the model sees a gradient instead of a flat wall.
if isnan(m.pos)
    J = 4.0;
    return
end
f = fieldnames(W);
J = 0; tw = 0;
for i = 1:numel(f)
    J  = J + W.(f{i}) * (m.(f{i}) / max(ref.(f{i}), 1e-9));
    tw = tw + W.(f{i});
end
J = J / tw;
if m.aborted
    J = J + 1.5*(1 - m.frac);
end
J = min(J, 4.0);
end

% -------------------------------------------------------------------------
function r = medianMetrics(ms)
ok = ~arrayfun(@(x) isnan(x.pos), ms);
if ~any(ok), r = ms(1); return; end
ms = ms(ok);
r  = ms(1);
for f = {'angP2P','rate','angHf','jerk','vel','pos','sway'}
    r.(f{1}) = median([ms.(f{1})]);
end
end

function s = describe(m, wheelR)
if isnan(m.pos)
    s = sprintf('%s, nothing usable', upper(m.why));
else
    if m.aborted
        tail = sprintf('   [%s at %.0f%%]', m.why, 100*m.frac);
    else
        tail = '';
    end
    s = sprintf(['travel %5.1f cm  sway %4.1f cm  wheel %4.2f rad/s  ' ...
                 'buzz %5.3f deg  chatter %4.1f V/s%s'], ...
                 100*wheelR*m.pos, 100*wheelR*m.sway, m.vel, ...
                 rad2deg(m.angHf), m.jerk, tail);
end
end

function showMetric(name, a, b, unit)
if a == 0 || isnan(a) || isnan(b)
    fprintf('  %-8s %10.4g %10.4g %9s  %s\n', name, a, b, '-', unit);
else
    fprintf('  %-8s %10.4g %10.4g %8.0f%%  %s\n', name, a, b, 100*(b/a-1), unit);
end
end

function [X0, Y0, C0] = seedPoints(inc, names, vars, carryX, carryY, carryC)
%SEEDPOINTS  Start each stage from the incumbent, plus any earlier episode
%   inside the new region. Re-running a known point costs half a minute.
row = cell2table(cellfun(@(nm) inc.(nm), names, 'UniformOutput', false), ...
                 'VariableNames', names);
X0 = row; Y0 = []; C0 = [];
if isempty(carryX), return; end
keep = true(height(carryX),1);
for i = 1:numel(names)
    v = carryX.(names{i});
    keep = keep & v >= vars(i).Range(1) & v <= vars(i).Range(2);
end
if any(keep)
    X0 = [carryX(keep,:); row];
    Y0 = [carryY(keep); NaN];        % NaN = evaluate the incumbent afresh
    if isempty(carryC)
        C0 = [-ones(sum(keep),1); NaN];
    else
        C0 = [carryC(keep,:); NaN];
    end
end
end

function tf = tooFar(win, abortAt)
%TOOFAR  Two triggers, because a robot can misbehave by shaking in place or
%   by quietly accelerating away.
tf = (max(win(:,1)) - min(win(:,1))) > abortAt.p2p || ...
     mean(abs(win(:,2))) > abortAt.vel;
end

function y = clip(x, lim)
y = min(max(x,-lim), lim);
end

% ---------------------------------------------------------------- console
function f = makeConsole(wheelR)
f = figure('Name','s03 autotune','NumberTitle','off','MenuBar','none', ...
           'Position',[40 520 460 340]);
setappdata(f,'stop',false);
setappdata(f,'hist',zeros(0,1));
setappdata(f,'wheelR',wheelR);
setappdata(f,'base',NaN);
uicontrol(f,'Style','pushbutton','Units','normalized', ...
    'Position',[0.02 0.88 0.20 0.10],'String','STOP','FontSize',12, ...
    'FontWeight','bold','BackgroundColor',[0.85 0.35 0.25], ...
    'Callback',@(s,e) setappdata(f,'stop',true));
ax = axes(f,'Units','normalized','Position',[0.13 0.13 0.83 0.70]);
grid(ax,'on'); box(ax,'on');
xlabel(ax,'episode'); ylabel(ax,'travel  [cm p-p]');
title(ax,'how far it wandered');
setappdata(f,'ax',ax);
drawnow
end

function setBaseline(f, refPos)
if ~isvalid(f), return; end
setappdata(f,'base', 100*getappdata(f,'wheelR')*refPos);
end

function noteEpisode(f, m)
if ~isvalid(f), return; end
h = [getappdata(f,'hist'); 100*getappdata(f,'wheelR')*m.pos];
setappdata(f,'hist',h);
ax = getappdata(f,'ax');
if ~isvalid(ax), return; end
cla(ax); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
plot(ax, 1:numel(h), h, 'o-', 'Color',[0.20 0.40 0.75], ...
     'MarkerFaceColor',[0.20 0.40 0.75], 'MarkerSize',4);
b = getappdata(f,'base');
if isfinite(b)
    yline(ax, b, '--', 'yours', 'Color',[0.70 0.25 0.20], 'LineWidth',1);
end
xlabel(ax,'episode'); ylabel(ax,'travel  [cm p-p]');
title(ax,'how far it wandered');
hold(ax,'off');
drawnow limitrate
end

function tf = stopped(f)
tf = ~isvalid(f) || getappdata(f,'stop');
end

function closeIfValid(f)
if isvalid(f), close(f); end
end

function safeShutdown(lnk)
try
    lnk.stop();
catch
end
try
    delete(lnk);
catch
end
end
