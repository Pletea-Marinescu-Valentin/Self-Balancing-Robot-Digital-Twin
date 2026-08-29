function build_pid_model(mdl)
%BUILD_PID_MODEL  Build the balancing PID model in Simulink, programmatically.
%
%   build_pid_model            -> creates 'sbr_pid'
%   build_pid_model('name')    -> creates a model with the given name
%
%   Why a script and not a shipped .slx: an .slx is a binary file. It cannot
%   be read in a diff and cannot be reviewed in a pull request. An .m builder
%   is versionable, reproducible, and editable from code. It is also the
%   recommended practice for generated models.
%
%   The structure built here mirrors the firmware control law exactly, so a
%   MIL run and a hardware run are directly comparable:
%
%       ref=0 --(+)--> [Kp] ------------------\
%                |                             (+)--> [Sat] --> u --> [SbrSerialIO]
%       angle ---/     [Ki/s] ----------------/                            |
%       rate  --------> [Kd] --(-)-----------/                             |
%                                                                          |
%       <---------------- angle, rate, position, ok ----------------------/
%
%   AFTER BUILDING, in the MATLAB System block set
%       "Simulate using" = Interpreted execution
%   and your serial port. Then Run.

arguments
    mdl (1,1) string = "sbr_pid"
end

Ts = 0.01;          % 100 Hz — start here, try 0.005 once it runs cleanly
Kp = 8.0;
Ki = 0.0;
Kd = 0.5;
UMax = 2.0;

if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

set_param(mdl, 'Solver', 'FixedStepDiscrete', ...
               'FixedStep', num2str(Ts), ...
               'StopTime', '60', ...
               'SolverType', 'Fixed-step');

% Soft real-time execution. Without pacing, Simulink runs as fast as it can
% and the exchange with the robot has no meaningful time base at all.
try
    set_param(mdl, 'EnablePacing', 'on', 'PacingRate', '1');
catch
    warning(['Could not enable Simulation Pacing automatically. ' ...
             'Turn it on manually: Run > Simulation Pacing > Enable pacing.']);
end

add = @(src, name, pos) add_block(src, mdl + "/" + name, 'Position', pos);

% -------------------------------------------------------------------- blocks
add('simulink/Sources/Constant',                     'ref',    [30  150  60  180]);
add('simulink/Math Operations/Sum',                  'err',    [110 145  130 185]);
add('simulink/Math Operations/Gain',                 'Kp',     [180  95  220 135]);
add('simulink/Discrete/Discrete-Time Integrator',    'Ki',     [180 155  220 195]);
add('simulink/Math Operations/Gain',                 'Kd',     [180 215  220 255]);
add('simulink/Math Operations/Sum',                  'usum',   [280 130  300 220]);
add('simulink/Discontinuities/Saturation',           'sat',    [340 155  380 195]);
add('simulink/User-Defined Functions/MATLAB System', 'robot',  [440 145  530 205]);
add('simulink/Signal Routing/Demux',                 'dmx',    [570 145  575 205]);
add('simulink/Sinks/Scope',                          'scope',  [680  95  720 135]);
add('simulink/Sinks/To Workspace',                   'log',    [680 215  730 255]);
add('simulink/Signal Routing/Mux',                   'mux',    [630  95  635 155]);

% ---------------------------------------------------------------- parameters
set_param(mdl+"/ref",   'Value', '0');
set_param(mdl+"/err",   'Inputs', '+-');
set_param(mdl+"/Kp",    'Gain', num2str(Kp));
set_param(mdl+"/Kd",    'Gain', num2str(Kd));
set_param(mdl+"/Ki",    'gainval', num2str(Ki), 'SampleTime', num2str(Ts));
set_param(mdl+"/usum",  'Inputs', '++-');
set_param(mdl+"/sat",   'UpperLimit', num2str(UMax), 'LowerLimit', num2str(-UMax));
set_param(mdl+"/dmx",   'Outputs', '4');
set_param(mdl+"/mux",   'Inputs', '2');
set_param(mdl+"/log",   'VariableName', 'simlog');
set_param(mdl+"/robot", 'System', 'SbrSerialIO');
try
    set_param(mdl+"/robot", 'SimulateUsing', 'Interpreted execution');
catch
    warning(['Set it manually in the "robot" block: ' ...
             'Simulate using = Interpreted execution.']);
end

% --------------------------------------------------------------------- wiring
c = @(a, b) add_line(mdl, a, b, 'autorouting', 'on');
c('ref/1',   'err/1');
c('dmx/1',   'err/2');        % measured angle
c('err/1',   'Kp/1');
c('err/1',   'Ki/1');
c('dmx/2',   'Kd/1');         % derivative from the gyro, not from the error
c('Kp/1',    'usum/1');
c('Ki/1',    'usum/2');
c('Kd/1',    'usum/3');
c('usum/1',  'sat/1');
c('sat/1',   'robot/1');
c('robot/1', 'dmx/1');
c('err/1',   'mux/1');
c('sat/1',   'mux/2');
c('mux/1',   'scope/1');
c('dmx/3',   'log/1');

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl);

fprintf(['Model "%s" created.\n\n' ...
         'Before you hit Run:\n' ...
         '  1. open the "robot" block and set your Port\n' ...
         '  2. check "Simulate using" = Interpreted execution\n' ...
         '  3. set Trim to the calibrated value (s03/s04 print it)\n' ...
         '  4. hold the robot near vertical at Start\n\n' ...
         'Run s01-s04 BEFORE this. If the .m scripts do not work, the model\n' ...
         'will not either, and it is much harder to debug.\n'], mdl);
end
