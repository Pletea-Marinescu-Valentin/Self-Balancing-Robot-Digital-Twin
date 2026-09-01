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
| `V_SUPPLY` | 10.0 | must equal the **measured** rail; see the note below |
| `V_LIMIT` | 8.0 | driver clamp |
| `MOTOR0_DIR` / `MOTOR1_DIR` | +1 / −1 | the motors face opposite ways |

**Command units.** With `PHASE_RESISTANCE = 0`, SimpleFOC runs in
torque/voltage mode and commands are in **volts** on the q axis. Set it above
zero and SimpleFOC computes `Uq = I·R`, so commands become **amps** — an
approximate torque interface. Measure between two phases with a multimeter
and halve the result. Switching later rescales every PID gain, so retune when
you do.

**`V_SUPPLY` has to match reality.** `BLDCDriver3PWM` computes duty as
`Ua / voltage_power_supply`, so if the define disagrees with the rail the
motor sees, every command is scaled by the ratio and every PID gain is
scaled with it. Measure the rail under load and put that number here.

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

## Control architecture

Two loops, both on the ESP32, cascaded. Rates come from `CTRL_HZ` and
`OUTER_DIV`.

```
  wheel speed, travel ──▶ OUTER (40 Hz, PI) ──▶ lean setpoint
                                                    │
                        tilt angle, gyro ──▶ INNER (200 Hz, PID) ──▶ volts
```

| Define | Value | Meaning |
|---|---|---|
| `OUTER_DIV` | 5 | outer loop at `CTRL_HZ / OUTER_DIV` = 40 Hz |
| `VEL_LPF_HZ` | 2.0 | wheel-speed filter feeding the outer loop |
| `FOC_VEL_TF` | 0.005 | SimpleFOC's own velocity filter, pinned not defaulted |
| `KV_DEFAULT` | 0.010 | rad of lean per rad/s of wheel speed |
| `KVI_DEFAULT` | 0.0026 | rad of lean per rad of wheel travel |
| `REF_LIMIT_DEFAULT` | 0.052 | 3°, the outer loop's entire authority |
| `KYAW_P_DEFAULT` | 0.0 | heading hold, off |
| `KYAW_D_DEFAULT` | 0.05 | heading damping, on |
| `U_FRIC_DEFAULT` | 0.0 | Coulomb feedforward, off |

**Why the outer loop is not optional.** The balance PID regulates one state.
Wheel position and speed never enter it, so they are not controlled at all: a
robot standing perfectly upright while crossing the room is, to that loop, a
correct answer. Any trim error, floor slope or mass asymmetry then integrates
without limit. Simulated with a 0.4° trim error, the inner loop alone drifts
37 m in 30 s and is still accelerating; adding `Kv` bounds it to under a metre
and `Kvi` brings it back to 0.12 m, with identical peak tilt.

**Why the rate ratio matters more than the rate.** An outer loop close in
bandwidth to the inner one turns a cascade into two controllers arguing, and
the result is a slow, large oscillation across the room. Roughly 5:1 is the
usual practice and is what `OUTER_DIV` encodes.

**Why the lean limit is the safety story.** Whatever the gains do, the outer
loop cannot ask for more tilt than `REF_LIMIT_DEFAULT`. Raise it only when the
log says the clamp is active a large fraction of the time; a bigger clamp is
more authority, not more margin.

**Why the signs come out positive.** Forward wheel state is built from the
same `MOTORn_DIR` constants used to drive the motors, and `initFOC()` aligns
each encoder so a positive q-axis voltage gives a positive `shaft_velocity`.
So `+u` means `+forward` by construction, and holding a positive lean
decelerates positive travel. `s02_balance.m` still measures the sign from the
log and says so, because a wrong sign here is a 300 m runaway in simulation.

**Yaw needs no extra sensor.** Differential wheel travel is a heading proxy.
Damping it removes the slow left-right wander; holding it is a preference and
defaults off.

## Wireless link (WiFi access point)

`PC_LINK_WIFI 1` in `config.h` moves the MATLAB link off USB. The motor node
brings up its own access point and serves the same binary protocol over TCP,
so nothing about the frame format changes.

| Define | Value |
|---|---|
| `WIFI_AP_SSID` | `sbr-robot` |
| `WIFI_AP_PASS` | `balancebot` |
| `WIFI_TCP_PORT` | 3333 |

The node's address is `192.168.4.1`, the ESP32 softAP default. Join the
network from the laptop, then pass the address instead of a COM port:

```matlab
lnk = SbrLink("192.168.4.1");
```

`SbrLink` picks the transport from the string: anything containing a dot is
treated as an address, everything else as a serial port.

This is only safe because the PID runs on the ESP32. Link latency is not in
the control loop, so a stalled packet costs points on a plot, not the robot.
Do not move a MATLAB-side control loop onto this.

Four things this arrangement needs, all of them already set:

- **`WDT_TIMEOUT_MS` becomes 1000 ms** when `PC_LINK_WIFI` is set. The 250 ms
  serial value is tighter than a WiFi retry burst, and would cut the motors
  for no reason. It is still a valid dead-man switch.
- **`setNoDelay(true)`** on the client and the server. Nagle's algorithm
  batches small writes, and every frame here is a small write.
- **`WiFi.setSleep(false)`.** ESP32 power saving parks the radio between
  beacons and adds 100 ms or more of latency.
- **Telemetry is dropped, never blocked.** `pcWrite` asks the socket with a
  zero-timeout `select()` and skips the frame if it is not writable. A
  blocking write inside the FOC loop would stall commutation:
  `NetworkClient::write()` gives up only after 10 retries of a 1 s select.

  Do **not** gate this on `availableForWrite()`. `WiFiClient` does not
  override it, so it returns `Print`'s default of `0` and every frame is
  silently discarded — the link connects, MATLAB waits, and not one byte
  arrives. It works on USB only because `HardwareSerial` does override it.

Watch `foc_hz` after switching. The radio stack shares the CPU, and the
number to compare against is what the same board reported over USB.

Only one client at a time; a second connection is accepted and immediately
closed so a stale session cannot wedge the port.

Flashing still needs the cable. `ArduinoOTA` would remove that too, and the
access point is already up for it.

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
