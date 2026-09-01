# Self-Balancing Robot — Digital Twin

Two-wheel self-balancing robot on brushless gimbal motors, with the control
loop on an ESP32 and MATLAB as the tuning console and data recorder.

Two cascaded loops run on the robot: a balance PID on the tilt angle at
200 Hz, and a slower wheel loop at 40 Hz that decides what tilt the balance
loop should aim for. MATLAB never sits inside either — it changes gains live,
logs, and plots. If the link drops, the robot keeps balancing.

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
firmware/motor_node/    FOC, encoders, cascaded control, safety, protocol
firmware/imu_node/      BNO055 sampling and streaming
matlab/                 bring-up, calibration, tuning and auto-tuning
docs/                   protocol and hardware reference
logs/                   .mat files written by the scripts
```

## Connecting

Over USB:

```matlab
lnk = SbrLink("COM4");
```

Over WiFi — the motor node runs its own access point (`sbr-robot` /
`balancebot`). Join it from the laptop, then:

```matlab
lnk = SbrLink("192.168.4.1");
```

Each script has a `PORT` line at the top; put either form there. Wireless
works here only because the PID runs on the ESP32: link latency is outside
the control loop, so a stalled packet costs points on a plot, not the robot.
Set `PC_LINK_WIFI` to 0 in `config.h` to go back to USB. Details and the
tuning that makes it reliable are in `docs/hardware.md`.

Flashing still needs the cable.

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

Sliders come in two groups, matching the two loops.

**Balance, inner, 200 Hz:** `Kp`, `Ki`, `Kd`, `Trim`. Trim moves the balance
point, which is what to reach for when the robot creeps steadily one way.

**Wheels, outer, 40 Hz:** `Kv`, `Kvi`, `Lean`, plus `YawD` and `Ufric`. `Kv`
damps wheel speed, `Kvi` brings the robot back to where it caught itself, and
`Lean` caps how far the outer loop may ask it to tilt — the outer loop's whole
authority, and the reason a bad gain there is survivable.

Tune the inner loop first, with `Kv` and `Kvi` at zero. A wheel loop wrapped
around a badly damped balance loop tells you nothing.

1. `Ki = 0`, `Kd = 0`. Raise `Kp` until it reacts briskly and starts to
   oscillate quickly. Note that value.
2. Drop `Kp` to about 60% of it, raise `Kd` until the oscillation goes.
3. Nudge `Trim` until the wheel-travel trace stops ramping one way.
4. Add `Ki` (0.5–2) last, only to remove a steady lean.

Then the outer loop, which is what keeps it on the table.

5. Raise `Kv` until the robot stops accelerating away and settles to a slow
   shuffle. Watch the wheel **speed** trace, not the angle.
6. Add `Kvi` until the travel trace returns towards zero instead of just
   holding still wherever it stopped.
7. Raise `Lean` only if the run report says the clamp is active a lot.
8. `YawD` if it wanders left-right; `Ufric` only if it hunts in place at an
   otherwise good tune, which is stiction rather than gains.

The report at the end also measures whether `Kv` has the right sign and says
so, rather than leaving you to discover it by watching the robot leave.

### 4. `s03_autotune.m` — let an agent finish the tune

Hand tuning gets you to "it stands". Getting from there to "it stands still"
means trading six numbers off against each other, which is a search problem
rather than an intuition problem.

```matlab
s03_autotune            % tune the robot
s03_autotune(true)      % rehearse against a model, no robot involved
```

Put your hand-found gains in `SEED` at the top first. The agent searches a
trust region around them, so a good seed is worth more than a big budget.

**It optimises for smoothness, not for small numbers.** Six measurements per
episode: peak-to-peak tilt, gyro rate, tilt energy above 0.8 Hz, command
chatter, wheel speed, wheel travel. The middle two are what separate "quiet"
from "visibly oscillating" — an RMS-only score is happy with a limit cycle.

**It does not need you standing over it.** Gains change live, the way the s02
sliders do, and the robot stays armed between episodes. Every episode is
watched on a rolling two-second window: the moment the wobble or the wheel
speed passes what your own baseline licensed, the episode is cut short, the
last known-good gains go back on immediately, and that candidate is scored by
how quickly it went wrong. The robot never gets the chance to build up an
oscillation or run off the bench. In rehearsal that is typically two cut-short
episodes and one fall in thirty-odd runs.

**`Ufric` is in the search.** A limit cycle around the setpoint is the classic
signature of Coulomb friction in the drive rather than of bad gains, and a
torque-mode gimbal motor has plenty of it. It is searched rather than simply
switched on, because over-compensated stiction makes a robot jitterier than
none at all.

Search is a trust region that widens when a stage improves and narrows when it
does not — the standard safe-BO construction, and what stops the agent from
throwing the robot across the room to find out what happens.

It finishes with a play-off: the agent's gains and yours, three runs each,
back to back, reported metric by metric. A noisy objective produces lucky
episodes, and repetition is the only defence.

**Why not reinforcement learning.** An RL agent learns a policy from scratch
and needs thousands of episodes; here every episode risks someone picking the
robot up. The controller is not unknown — it is a cascade whose structure is
right and whose numbers are wrong. That is black-box optimisation against a
noisy objective, which is what Bayesian optimisation is for, and what the
robotics literature uses to tune controllers on hardware.

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

**A balance loop alone cannot keep the robot in one place.** It regulates the
tilt angle and nothing else, so wheel position and speed are uncontrolled
states: upright and travelling is, to that loop, a perfect answer. No `Kp`,
`Ki` or `Kd` fixes it — the missing feedback has to be added, not tuned. That
is what the wheel loop is.

**Never gate a WiFi write on `availableForWrite()`.** `WiFiClient` does not
override it, so it returns `Print`'s default of `0` and every frame is
dropped in silence. The link connects, the board looks dead, and nothing in
the error message points at the sender. `HardwareSerial` does override it,
which is why the same code worked on USB.

**Pace any MATLAB loop that writes to the port.** An unpaced loop issues
thousands of commands a second, floods the link and starves its own reads —
the board looks dead while it is perfectly healthy.
