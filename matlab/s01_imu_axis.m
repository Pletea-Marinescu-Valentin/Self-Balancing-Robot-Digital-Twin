%% s01_imu_axis — one-time IMU axis identification
%  Run once per physical build. Prints the two lines to paste into config.h.

clear; clc; close all;

PORT     = "192.168.4.1";      % or "COM4" for USB
TILT_SEC = 4;

lnk = SbrLink(PORT);
cleanupObj = onCleanup(@() delete(lnk));

fprintf('Set the robot down and leave it STILL for ~10 s.\n');
fprintf('On the imu_node serial, watch "gyr=" climb to 3.\n');
fprintf('Press Enter when it has.\n');
pause;

fprintf('\nNow TILT the robot back and forth about its balance axis,\n');
fprintf('~20 deg each way. Press Enter, then tilt for %d s.\n', TILT_SEC);
pause;
lnk.flushInput();

tc = tic; E = []; G = []; TT = [];
while toc(tc) < TILT_SEC
    d = lnk.readLatest();
    if ~isempty(d)
        E(end+1,:) = d.eul;  G(end+1,:) = d.gyr;  TT(end+1,1) = toc(tc); %#ok<SAGROW>
    end
end
assert(size(E,1) > 20, 'No telemetry during the tilt. Run s00_motor_check first.');

spans = rad2deg(max(E) - min(E));
fprintf('\n  Euler movement:  heading %.1f  roll %.1f  pitch %.1f  [deg]\n', spans);

CAND = [2 3];
[bestSpan, iCand] = max(spans(CAND));
angIdx = CAND(iCand);

if bestSpan < 5
    lnk.stop();
    if all(all(E == 0))
        error('SBR:imuDead', ['Every Euler and gyro channel reads exactly 0.\n' ...
            'The link is fine but the BNO055 is not fusing. Plug the IMU\n' ...
            'ESP32 into the laptop, open its serial at 115200, and check the\n' ...
            'status line: cfg=0 means the chip never entered fusion mode.']);
    end
    error('SBR:axis', ['Neither roll nor pitch moved more than %.1f deg.\n' ...
        'Tilt further, or check the imu_node status line.'], bestSpan);
end

cc = @(a,b) ((a-mean(a))' * (b-mean(b))) / ...
            (norm(a-mean(a)) * norm(b-mean(b)) + eps);
dE = diff(E(:,angIdx)) ./ max(diff(TT), eps);
r  = arrayfun(@(j) cc(dE, (G(1:end-1,j) + G(2:end,j))/2), 1:3);
fprintf('  gyro match:      x %+.2f  y %+.2f  z %+.2f\n', r);

[~, gyrIdx] = max(abs(r));
gyrSign = sign(r(gyrIdx));

eulNames = {'heading','roll','pitch'};
fprintf('\n  balance angle = Euler %d (%s), span %.1f deg\n', ...
        angIdx-1, eulNames{angIdx}, bestSpan);
fprintf('  rate          = gyro %d, sign %+d, correlation %.2f\n\n', ...
        gyrIdx-1, gyrSign, r(gyrIdx));

if abs(r(gyrIdx)) < 0.8
    fprintf(2, ['  WARNING: correlation %.2f is weak. Tilt more smoothly and\n' ...
                '  rerun before trusting this pairing.\n\n'], abs(r(gyrIdx)));
end

lnk.imuAxis(angIdx-1, gyrIdx-1, +1, gyrSign);
lnk.stop();

fprintf('Applied for this session. To make it permanent, put these in\n');
fprintf('firmware/motor_node/config.h and reflash:\n\n');
fprintf('    #define IMU_ANGLE_IDX    %d\n', angIdx-1);
fprintf('    #define IMU_ANGLE_SIGN (+1.0f)\n');
fprintf('    #define IMU_GYRO_IDX     %d\n', gyrIdx-1);
fprintf('    #define IMU_GYRO_SIGN  (%+.1ff)\n\n', gyrSign);
fprintf('If the robot drives itself over instead of catching itself,\n');
fprintf('flip IMU_ANGLE_SIGN and leave everything else alone.\n');
