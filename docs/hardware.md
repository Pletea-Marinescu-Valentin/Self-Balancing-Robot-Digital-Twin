# Hardware reference

Everything in `firmware/motor_node/config.h`, explained. Change hardware,
change that file, nothing else.

## Pinout — MKS DUAL FOC v3.2 Plus on an ESP32 Dev Module

| Signal | Pins |
|---|---|
| Motor 0 phases + enable | 32, 33, 25, 22 |
| Motor 1 phases + enable | 26, 27, 14, 12 |
| I2C bus 0 (SDA, SCL) | 19, 18 |
| I2C bus 1 (SDA, SCL) | 23, 5 |
| UART2 to the IMU node | RX 16, TX 17 |

`M1_EN` is GPIO12, a boot strapping pin (MTDI). If the board ever refuses to
boot with the shield powered, that is the first suspect.

## Encoder to motor pairing

```
M0_ENC_BUS  1     motor 0 reads the AS5600 on bus 1
M1_ENC_BUS  0     motor 1 reads the AS5600 on bus 0
```

This is a wiring fact, measured, not a preference. The AS5600 address is
0x36 and is not configurable, so the two sensors cannot share a bus.

Getting the pairing wrong is not a small error. FOC commutates each motor
from the angle it believes is its own rotor position. Pair a motor with the
other motor's encoder and the stator field never aligns: the motor does not
turn, and it draws full current the instant a command arrives. That current
collapses the supply and floods the sensor bus with noise, so the first
visible symptom is usually the IMU dying, several boards away.

`s00_motor_check.m` verifies the pairing before anything is armed.

## Motor and driver

| Define | Value | Meaning |
|---|---|---|
| `POLE_PAIRS` | 7 | magnets / 2 |
| `PHASE_RESISTANCE` | 0.0 | see below |
| `V_SUPPLY` | 12.0 | actual supply voltage |
| `V_LIMIT` | 8.0 | driver clamp |
| `MOTOR0_DIR` / `MOTOR1_DIR` | +1 / −1 | the motors face opposite ways |

**Command units.** With `PHASE_RESISTANCE = 0`, SimpleFOC runs in
torque/voltage mode and commands are in **volts** on the q axis. Set it above
zero and SimpleFOC computes `Uq = I·R`, so commands become **amps** — an
approximate torque interface. Measure between two phases with a multimeter
and halve the result. Switching later rescales every PID gain, so retune when
you do.

**Torque mode is correct for balancing, and the wheel will feel loose.** A
position-controlled motor holds its shaft stiffly; a balancing robot needs
free wheels it can drive. Softness at zero command is the design, not a
fault.

## IMU axis

```
IMU_ANGLE_IDX    Euler channel: 0 heading, 1 roll, 2 pitch
IMU_GYRO_IDX     gyro channel:  0 x, 1 y, 2 z
IMU_ANGLE_SIGN   flip if the robot drives itself over instead of catching itself
IMU_GYRO_SIGN    independent of the angle sign, and often different
```

These are power-on defaults only. `s01_imu_axis.m` measures the true pair and
pushes it over the link at runtime, so bring-up never needs a reflash. Pin
the measured values here once they are confirmed.

The two signs are independent on purpose: the BNO055 Euler and gyro
conventions do not always agree on a given axis, and a rate term with the
wrong sign turns damping into positive feedback.

## Loop rates and safety

| Define | Value | Meaning |
|---|---|---|
| `CTRL_HZ` | 200 | control loop rate |
| `TELEM_DIV` | 4 | telemetry at `CTRL_HZ / TELEM_DIV` = 50 Hz |
| `SERIAL_BAUD` | 115200 | USB to MATLAB |
| `IMU_BAUD` | 460800 | UART between the two ESP32s |
| `IMU_TIMEOUT_MS` | 50 | ~5 missed samples before the IMU is declared dead |
| `WDT_TIMEOUT_MS` | 250 | no valid PC command cuts the motors |
| `TILT_LIMIT_RAD` | 0.60 | ~34°, latched stop |

The IMU arrives at 100 Hz while the loop runs at 200 Hz, so every second
cycle reuses a sample. That is fine: the derivative term reads the gyro
directly and is never differentiated.

`IMU_BAUD` is a direct wire between two ESP32s with no USB bridge in the
path, which is why it stays high while the USB link does not.

## Health numbers

Read these from `s00_motor_check.m`:

| Reading | Healthy | Meaning |
|---|---|---|
| `foc_hz` | > 1000 | FOC loop rate. Below ~800 the commutation is coarse and the motors buzz |
| `foc_hz` | **negative** | sentinel: `initFOC()` failed, the board refuses to arm |
| `imu_hz` | ~100 | frames arriving from the sensor node |
| `imu_ok` | 1 | fresh frames **and** a BNO055 confirmed to be fusing |
| `imu_link` | 1 | frames pass CRC, whatever they contain |

`imu_link = 1` with `imu_ok = 0` means the cable is fine and the sensor is
not. They are separate bits for that reason.

## The IMU node's own serial

Plug the sensor ESP32 into a computer and open its port at 115200. It prints
one line a second:

```
ok=1 cfg=1  101 Hz  i2c_fail=0  eul=[ 354.6  2.6  -44.2] deg  calib sys=0 gyr=3
```

- `cfg=0` — the BNO055 never entered fusion mode. Nothing downstream is real.
- `i2c_fail` — should stay 0. A rising count is motor noise reaching the
  sensor bus.
- `calib gyr` — must reach 3 before balancing. The gyro calibrates only while
  the robot is held still, and an uncalibrated gyro feeds the D term a
  constant false rate.
- `calib sys` stays 0 in IMUPLUS mode. That is expected: there is no
  magnetometer to complete a full system calibration.

IMUPLUS (0x08) is deliberate. The BLDC motors sit centimetres from the
sensor and make magnetic heading meaningless; NDOF would let that corruption
leak into the fused attitude, including the axis being balanced on. Roll and
pitch are gravity-referenced and identical in both modes.
