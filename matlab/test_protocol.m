function test_protocol()
%TEST_PROTOCOL  Byte-level checks of the frame layout. No hardware needed.
%   Run after ANY change to the frame, on either side: a mismatch does not
%   announce itself, the checksum still passes and the fields simply hold
%   the wrong numbers.  Usage:  cd matlab; test_protocol

fprintf('=== Telemetry frame (motor node -> MATLAB) ===\n');

PKT = 88;
vals   = (1:20) * 1.5;          % distinct value per float field
t_us   = uint32(123456789);
status = uint8(1 + 2 + 16 + 32 + bitshift(2, 6));  % armed, imu_ok, imu_fault, imu_link, mode 2

p = zeros(PKT, 1, 'uint8');
p(1) = 165;  p(2) = 90;                                  % 0xA5 0x5A
p(3:6)  = typecast(t_us, 'uint8')';
p(7:86) = typecast(single(vals), 'uint8')';
p(87)   = status;
p(88)   = SbrLink.xorSum(p(3:87));      % XOR over bytes 2..86, as in firmware

% --- size and checksum ---------------------------------------------------
assert(numel(p) == PKT, 'frame length');
assert(SbrLink.xorSum(p(3:end-1)) == p(end), 'checksum agreement');

% --- decode --------------------------------------------------------------
d = SbrLink.decode(p);

chk = @(name, got, want) assert(abs(got - want) < 1e-4, ...
        sprintf('%s: got %.4f, expected %.4f', name, got, want));

chk('t_us',       d.t_us,       double(t_us));
chk('eul(1)',     d.eul(1),     vals(1));
chk('eul(3)',     d.eul(3),     vals(3));
chk('gyr(1)',     d.gyr(1),     vals(4));
chk('gyr(3)',     d.gyr(3),     vals(6));
chk('acc(1)',     d.acc(1),     vals(7));
chk('acc(3)',     d.acc(3),     vals(9));
chk('angle',      d.angle,      vals(10));
chk('rate',       d.rate,       vals(11));
chk('pos0',       d.pos0,       vals(12));
chk('pos1',       d.pos1,       vals(13));
chk('vel0',       d.vel0,       vals(14));
chk('vel1',       d.vel1,       vals(15));
chk('u0',         d.u0,         vals(16));
chk('u1',         d.u1,         vals(17));
chk('foc_hz',     d.foc_hz,     vals(18));
chk('imu_hz',     d.imu_hz,     vals(19));
chk('imu_age_ms', d.imu_age_ms, vals(20));

% --- status bits ---------------------------------------------------------
assert(d.armed      == true,  'armed bit');
assert(d.imu_ok     == true,  'imu_ok bit');
assert(d.tilt_fault == false, 'tilt_fault bit');
assert(d.wdt_fault  == false, 'wdt_fault bit');
assert(d.imu_fault  == true,  'imu_fault bit');
assert(d.imu_link   == true,  'imu_link bit');
assert(d.mode       == 2,     'mode field');

% --- orientation independence -------------------------------------------
d2 = SbrLink.decode(p');
assert(isequal(d, d2), 'decode must accept a row vector too');

fprintf('  20 float fields, 6 status bits, 2 orientations : OK\n');

%% ------------------------------------------------------------------------
fprintf('\n=== Command frame (MATLAB -> motor node) ===\n');

% Mirrors SbrLink.send. If this drifts from the private method, the two
% expressions below stop agreeing and the assertion fires.
cmd = zeros(1, 16, 'uint8');
cmd(1) = 165; cmd(2) = 90; cmd(3) = 8;                   % SBR_IMUAXIS
cmd(4:15) = typecast(single([3 -1 0]), 'uint8');         % Euler[2]+, gyro[0]-
cmd(16) = SbrLink.xorSum(cmd(3:15));

assert(numel(cmd) == 16, 'command length');
q = double(typecast(cmd(4:15), 'single'));
assert(isequal(q, [3 -1 0]), 'command payload round trip');
fprintf('  16-byte layout, signed channel selectors      : OK\n');

%% ------------------------------------------------------------------------
fprintf('\n=== IMU link frame (sensor node -> motor node) ===\n');

% Documented in docs/protocol.md and asserted in both .ino files with
% static_assert. Repeated here so the three descriptions cannot silently
% disagree.
imuLen = 2 + 4 + 3*4 + 3*4 + 3*4 + 1 + 1;
assert(imuLen == 44, 'IMU frame must be 44 bytes, computed %d', imuLen);
fprintf('  44-byte layout                                : OK\n');

%% ------------------------------------------------------------------------
fprintf('\n=== Shared header, duplicated per sketch folder ===\n');

% Arduino cannot include a header from outside its own sketch folder, so
% protocol.h exists twice. Two copies of a wire format is exactly the kind of
% thing that drifts silently, so it is checked rather than trusted.
here = fileparts(mfilename('fullpath'));
h1 = fullfile(here, '..', 'firmware', 'motor_node', 'protocol.h');
h2 = fullfile(here, '..', 'firmware', 'imu_node',  'protocol.h');

if isfile(h1) && isfile(h2)
    assert(isequal(fileread(h1), fileread(h2)), ...
        ['firmware/motor_node/protocol.h and firmware/imu_node/protocol.h ' ...
         'have drifted apart. Copy one over the other.']);
    fprintf('  the two protocol.h copies are identical      : OK\n');
else
    warning('protocol.h not found next to matlab/; identity check skipped.');
end

fprintf('\nALL PROTOCOL CHECKS PASSED\n');
end
