%% s00_motor_check — do the wheels turn? Run with the WHEELS OFF THE GROUND.

clear; clc; close all;

PORT     = "COM4";      % or "192.168.4.1" for WiFi
U_TEST   = 3.0;
TILT_MAX = 3.0;
HOLD     = 1.2;
SETTLE   = 0.6;

lnk = SbrLink(PORT);
cleanupObj = onCleanup(@() delete(lnk));

fprintf('=== Board health ===\n');
fprintf('  waiting for the first valid frame');
info = lnk.diagnose(15);
fprintf(' (%.1f s, %d bytes)\n', info.elapsed, info.nBytes);

if isempty(info.frame)
    msg = sprintf('Port %s opened, but no valid frame arrived in %.0f s.\n', ...
                  PORT, info.elapsed);
    if info.nBytes == 0
        msg = [msg sprintf([ ...
            '  Not one byte was received, so the board is not transmitting.\n' ...
            '   1. Power-cycle it: unplug USB *and* 12 V, then plug back in.\n' ...
            '   2. Check nothing else holds the port (Arduino Serial Monitor,\n' ...
            '      or a stale MATLAB serialport - run "clear all").\n' ...
            '   3. Open the Serial Monitor at 115200 and reset the board.\n' ...
            '      Gibberish means it IS talking; silence means it is not.\n'])];
    elseif strlength(info.text) > 0
        msg = [msg sprintf([ ...
            '  The board sent TEXT, not telemetry. This firmware prints text\n' ...
            '  only when initFOC() fails:\n\n      %s\n\n'], info.text)];
    else
        msg = [msg sprintf([ ...
            '  %d bytes arrived but none formed a valid frame, so the two sides\n' ...
            '  disagree about the format. On USB that is SERIAL_BAUD in config.h\n' ...
            '  vs the baud in SbrLink.m; both want 115200. On WiFi it means the\n' ...
            '  firmware and protocol.h are out of step - reflash.\n'], ...
            info.nBytes)];
    end
    error('SBR:noTelemetry', '%s', msg);
end

d = info.frame; tw = tic;
while toc(tw) < 1.5
    dn = lnk.readLatest();
    if ~isempty(dn), d = dn; end
end

fprintf('  foc_hz  = %8.1f\n', d.foc_hz);
fprintf('  imu_hz  = %8.1f   imu_ok = %d   imu_link = %d\n', ...
        d.imu_hz, d.imu_ok, d.imu_link);

if d.foc_hz < 0
    error('SBR:align', ['initFOC() FAILED on at least one motor; the firmware\n' ...
        'refuses to arm. Open the Serial Monitor at 115200 and reset to see\n' ...
        'which. Usual causes: 12 V absent at power-up, a magnet not centred\n' ...
        'over its AS5600, or M0_ENC_BUS/M1_ENC_BUS wrong in config.h.']);
end
if d.foc_hz < 800
    fprintf(2, '  WARNING: FOC loop at %.0f Hz (want > 800).\n', d.foc_hz);
end
if ~d.imu_ok
    error('SBR:imu', ['imu_ok = 0, so the firmware will not arm. Check the\n' ...
        'crossed UART wiring (16<->17) and the shared ground.']);
end
fprintf('  OK.\n');

lnk.trim(d.angle);
lnk.limits(U_TEST + 1, TILT_MAX);
lnk.setpoint(0);
lnk.mode(1);
lnk.flushInput();

fprintf('  arming');
armedOk = false; ta = tic;
while toc(ta) < 3
    lnk.arm(true);
    lnk.torque(0, 0);
    dn = lnk.readLatest();
    if ~isempty(dn) && dn.armed && ~dn.wdt_fault && ~dn.tilt_fault
        armedOk = true; break
    end
    pause(0.02);
end
fprintf(' %s (%.2f s)\n', string(armedOk), toc(ta));

if ~armedOk
    lnk.stop();
    if isempty(dn)
        error('SBR:arm', 'Telemetry stopped while arming - the board reset.');
    end
    error('SBR:arm', ['No armed=1 within 3 s. Last state: armed=%d tilt=%d ' ...
        'wdt=%d imu_ok=%d'], dn.armed, dn.tilt_fault, dn.wdt_fault, dn.imu_ok);
end

ok = true;  died = '';  cause = '';

fprintf('\n=== Ramp on motor 0 ===\n');
fprintf('%8s %10s %8s  %s\n', 'u', 'wheel0', 'frames', 'link');

uLast = 0;  uDied = NaN;
for u = 0.5 : 0.5 : U_TEST
    r = holdAndMeasure(lnk, u, 0, 0.8);
    if r.nFrames == 0
        fprintf(2, '%8.1f %10s %8d  LINK LOST\n', u, '-', 0);
        died  = sprintf('link survived u = %.1f, died at u = %.1f', uLast, u);
        cause = 'brownout';  uDied = u;  ok = false;
        break
    end
    fprintf('%8.1f %10.2f %8d  %s\n', u, r.v0, r.nFrames, r.flags);
    if ~isempty(r.fault)
        died = r.fault;  cause = r.cause;  uDied = u;  ok = false;
        break
    end
    uLast = u;
    holdAndMeasure(lnk, 0, 0, SETTLE);
end

cases = { 'motor 0 forward',  U_TEST,       0
          'motor 0 reverse', -U_TEST,       0
          'motor 1 forward',       0,  U_TEST
          'motor 1 reverse',       0, -U_TEST
          'both forward',     U_TEST,  U_TEST };

if ~isnan(uDied)
    cases = {};
else
    fprintf('\n=== Spin test at u = %.1f ===\n', U_TEST);
    fprintf('%-17s %8s %8s %7s %8s  %s\n', ...
            'case', 'wheel0', 'wheel1', 'frames', 'foc_hz', 'link');
end

for i = 1:size(cases, 1)
    r = holdAndMeasure(lnk, cases{i,2}, cases{i,3}, HOLD);
    if r.nFrames == 0
        fprintf(2, '%-17s %8s %8s %7d %8s  NO TELEMETRY\n', cases{i,1}, '-', '-', 0, '-');
        died = 'the board stopped sending telemetry'; cause = 'silent';
        ok = false; break
    end
    fprintf('%-17s %8.2f %8.2f %7d %8.0f  %s\n', cases{i,1}, ...
            r.v0, r.v1, r.nFrames, r.focHz, r.flags);
    if ~isempty(r.fault)
        died = r.fault; cause = r.cause; ok = false; break
    end
    if cases{i,2} ~= 0 && abs(r.v0) < 1.0, ok = false; end
    if cases{i,3} ~= 0 && abs(r.v1) < 1.0, ok = false; end
    holdAndMeasure(lnk, 0, 0, SETTLE);
end

holdAndMeasure(lnk, 0, 0, 0.3);
lnk.stop();

fprintf('\n=== Verdict ===\n');
if ok
    fprintf('  PASS. Both wheels respond at %.1f V in both directions.\n', U_TEST);
elseif ~isempty(died)
    fprintf(2, '  STOPPED EARLY - not a verdict on the motors:\n    %s\n\n', died);
    switch cause
        case 'brownout'
            fprintf(2, ['  Watch the Serial Monitor at 115200 while it fails.\n' ...
                '  A boot banner means the board RESET (check the 12 V rail and\n' ...
                '  how the ESP32 is fed). Continuing gibberish means only MATLAB\n' ...
                '  lost the link - a host or EMI problem, not a robot one.\n']);
        case {'silent', 'disarm'}
            fprintf(2, '  The board went away mid-test: brownout or reset.\n');
        case 'imu'
            fprintf(2, ['  The IMU node stopped being heard and the firmware cut\n' ...
                '  the motors. Check the shared ground and the IMU supply.\n']);
        case 'wdt'
            fprintf(2, ['  MATLAB went quiet for longer than WDT_TIMEOUT_MS and\n' ...
                '  the firmware cut the motors, as designed. Close heavy\n' ...
                '  background work; over WiFi, check the link is not dropping.\n']);
        case 'tilt'
            fprintf(2, '  Tilt limit tripped; raise TILT_MAX at the top.\n');
    end
else
    fprintf(2, ['  FAIL. A commanded wheel did not turn while the link stayed\n' ...
        '  healthy, so this is the actuator.\n' ...
        '   1. One direction only on the same motor: phase wiring or magnet.\n' ...
        '   2. One motor never: swap the two motor connectors. If the fault\n' ...
        '      follows the connector it is the driver, else the motor.\n' ...
        '   3. Raise U_TEST here (V_LIMIT in config.h is 8).\n']);
end

function r = holdAndMeasure(lnk, u0, u1, secs)
    r = struct('v0',0, 'v1',0, 'nFrames',0, 'focHz',NaN, 'flags','', ...
               'fault','', 'cause','');
    t = tic;  kNext = 0;  Ts = 0.01;
    while toc(t) < secs
        while toc(t) < kNext * Ts, pause(0.002); end
        kNext = kNext + 1;

        lnk.torque(u0, u1);
        d = lnk.readLatest();
        if isempty(d), continue; end

        r.nFrames = r.nFrames + 1;
        r.focHz   = min([r.focHz, d.foc_hz]);
        r.v0      = max(r.v0, abs(d.vel0));
        r.v1      = max(r.v1, abs(d.vel1));
        r.flags   = sprintf('armed=%d imu_ok=%d', d.armed, d.imu_ok);

        if ~d.armed
            r.cause = 'disarm';
            r.fault = 'the board reports armed=0';
        elseif ~d.imu_ok
            r.cause = 'imu';
            r.fault = 'the IMU node stopped being heard';
        elseif d.wdt_fault
            r.cause = 'wdt';
            r.fault = 'the command watchdog fired';
        elseif d.tilt_fault
            r.cause = 'tilt';
            r.fault = 'the tilt limit tripped';
        end
        if ~isempty(r.fault), return; end
    end
end
