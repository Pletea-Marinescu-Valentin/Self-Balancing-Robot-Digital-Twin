# Self-Balancing Robot — Digital Twin

Two-wheel self-balancing robot on brushless gimbal motors, with the control
loop on an ESP32 and MATLAB as the tuning console and data recorder.

The PID runs on the robot at a fixed 200 Hz. MATLAB never sits inside the
control loop — it changes gains live, logs, and plots. If the USB link drops,
the robot keeps balancing.

## Hardware

| Part | Detail |
|---|---|
| Motor node | ESP32 Dev Module + MKS DUAL FOC v3.2 Plus shield |
| Motors | 2x BLDC gimbal, 7 pole pairs, 12 V |
| Encoders | 2x AS5600 over I2C, one per hardware bus |
| Sensor node | second ESP32 + BNO055 |
| Link | UART2 between the nodes, USB from the motor node to MATLAB |

The BNO055 is **not** on the motor board. It has its own ESP32, which streams
attitude over UART2. The two boards must share a ground.

## Layout

```
firmware/motor_node/    FOC, encoders, PID, safety, MATLAB protocol
firmware/imu_node/      BNO055 sampling and streaming
matlab/                 bring-up, calibration and tuning scripts
docs/                   protocol and hardware reference
logs/                   .mat files written by the scripts
```

## Bring-up order

Flash both nodes once, then run the scripts in order. Each one has to pass
before the next is meaningful.

### 1. `s00_motor_check.m` — do the wheels turn?

Wheels off the ground. Checks the link, verifies FOC alignment, arms, ramps
the command, then drives each motor alone in both directions and reports the
measured wheel speed.

It distinguishes a wheel that refused to move from a board that stopped
talking. Those look identical from MATLAB and need opposite fixes.

### 2. `s01_imu_axis.m` — which channel is the balance angle?

Run once per physical build. Hold the robot still until the BNO055 gyro
calibration reaches 3, then tilt it back and forth for four seconds.

The script records all six IMU channels, picks the Euler channel that
actually moved, and pairs it with the gyro axis that tracks its derivative.
It prints the four `#define` lines to paste into `config.h`.

Heading is never a candidate: a two-wheel robot does not tip about yaw, and
heading wraps at 360°, which would win any largest-movement test outright.

### 3. `s02_balance.m` — tune it

Lay the robot down and run the script. It arms itself when the balance angle
comes within the capture window and disarms when it leaves, so you start from
the floor, lift the robot, and it catches itself. Drop it and lift it again —
the session, the sliders and the log all survive.

Four sliders: `Kp`, `Ki`, `Kd`, and `Trim`. Trim moves the balance point,
which is what to reach for when the robot creeps steadily in one direction.

Tuning order:

1. `Ki = 0`, `Kd = 0`. Raise `Kp` until it reacts briskly and starts to
   oscillate quickly. Note that value.
2. Drop `Kp` to about 60% of it, raise `Kd` until the oscillation goes.
3. Nudge `Trim` until it stops creeping.
4. Add `Ki` (0.5–2) last, only to remove a steady lean.

### `test_protocol.m`

Compares the two copies of `protocol.h` byte for byte and checks the frame
sizes. The header is duplicated because Arduino cannot include across sketch
folders, so nothing but this test stops the two nodes from drifting apart.

## Operating modes

The motor node has three, selected at runtime:

| Mode | Who computes the command |
|---|---|
| 0 idle | nobody; motors off |
| 1 direct | MATLAB sends `u` per wheel. Used by `s00_motor_check` |
| 2 pid | the ESP32, at `CTRL_HZ`. Used by `s02_balance` |

## Safety interlocks

Motors run only when **all** of these hold:

- FOC alignment succeeded at boot
- armed explicitly from MATLAB
- fresh IMU data within `IMU_TIMEOUT_MS`
- no latched tilt fault
- a valid command within `WDT_TIMEOUT_MS`

Tilt, watchdog and IMU faults latch. Arming is the only thing that clears
them, which makes an intermittent fault visible instead of self-healing.

## If the loop polarity is inverted

The robot drives itself over harder as it tips instead of catching itself.
Flip `IMU_ANGLE_SIGN` in `config.h` and change nothing else.

## Notes worth keeping

**Never call `motor.enable()` from the control loop.** SimpleFOC's `enable()`
and `disable()` both call `driver->setPwm(0,0,0)`. Zero duty on all three
phases turns on all three low-side FETs, which shorts the winding — that is a
brake, not "off". Calling it once per control cycle brakes both wheels
`CTRL_HZ` times a second: audible crackling and almost no usable torque. The
firmware gates the driver on the safe/unsafe **edge** for this reason.

**A successful I2C read is not a working sensor.** A BNO055 left in CONFIG
mode acknowledges every read and answers `0x00`, so a bare read-succeeded
check reports a healthy sensor while streaming nothing but zeros. `imu_node`
reads `OPR_MODE` back after configuring it and re-checks it at 10 Hz.

**Serial at 115200 with 50 Hz telemetry.** 4.4 kB/s against an 11.5 kB/s
budget. Higher rates bought nothing and added a variable to every debugging
session. The stream is binary: opening a serial monitor on it shows
gibberish at any baud, and that gibberish means the board is transmitting.

**Pace any MATLAB loop that writes to the port.** An unpaced loop issues
thousands of commands a second, floods the link and starves its own reads —
the board looks dead while it is perfectly healthy.
