# Serial protocols

Two independent binary links, both little-endian (ESP32 and x86 are both LE,
so `memcpy` / `typecast` work directly with no conversion).

```
  [ imu_node ] --UART2, 460800, 100 Hz--> [ motor_node ] --USB, 115200, 50 Hz--> [ MATLAB ]
   BNO055                                  2x FOC + 2x AS5600
```

Checksum on both links is an **XOR** over every byte between the header and
the checksum itself, exclusive of neither end's sync bytes and inclusive of
the status byte.

The two links use **different sync words** on purpose. If a cable ends up in
the wrong header, the frames are rejected instead of being silently decoded
into plausible-looking garbage.

---

## Link A · IMU node → motor node · 44 bytes · 100 Hz · 460800 baud

Wiring is crossed: `imu TX(17) → motor RX(16)`, `imu RX(16) ← motor TX(17)`,
plus a **shared ground**.

| Offset | Type     | Field    | Unit  | Notes |
|-------:|----------|----------|-------|-------|
| 0      | `uint8`  | sync0    | —     | `0xA7` |
| 1      | `uint8`  | sync1    | —     | `0x5B` |
| 2–5    | `uint32` | `t_us`   | µs    | `micros()` on the IMU node |
| 6–17   | `float×3`| `eul`    | rad   | `[0]` heading `[1]` roll `[2]` pitch |
| 18–29  | `float×3`| `gyr`    | rad/s | `[0]` x `[1]` y `[2]` z |
| 30–41  | `float×3`| `acc`    | m/s²  | `[0]` x `[1]` y `[2]` z |
| 42     | `uint8`  | `status` | —     | see below |
| 43     | `uint8`  | `crc`    | —     | XOR over bytes 2…42 |

### `status`

| Bits | Meaning |
|-----:|---------|
| 0    | sensor read succeeded |
| 1    | reserved |
| 2–3  | BNO055 system calibration, 0…3 |
| 4–5  | gyroscope calibration, 0…3 |
| 6–7  | accelerometer calibration, 0…3 |

The IMU node sends **all three** Euler angles and **all three** gyro axes and
never decides which one matters. That decision belongs to the motor node,
which takes it at runtime from MATLAB — which is why this firmware is written
once and never reflashed during bring-up.

Units are set explicitly in `UNIT_SEL` (register `0x3B` = `0x00`) rather than
trusted as a power-on default, because bit 7 of that register flips the sign
convention of pitch, and a silent change there would invert the balance axis.

---

## Link B · motor node → MATLAB · 88 bytes · `CTRL_HZ / TELEM_DIV` (50 Hz) · 115200 baud

| Offset | Type      | Field        | Unit  | Notes |
|-------:|-----------|--------------|-------|-------|
| 0      | `uint8`   | sync0        | —     | `0xA5` |
| 1      | `uint8`   | sync1        | —     | `0x5A` |
| 2–5    | `uint32`  | `t_us`       | µs    | `micros()` on the motor node |
| 6–17   | `float×3` | `eul`        | rad   | forwarded unchanged |
| 18–29  | `float×3` | `gyr`        | rad/s | forwarded unchanged |
| 30–41  | `float×3` | `acc`        | m/s²  | forwarded unchanged |
| 42–45  | `float`   | `angle`      | rad   | **selected** channel, sign applied |
| 46–49  | `float`   | `rate`       | rad/s | **selected** channel, sign applied |
| 50–53  | `float`   | `pos0`       | rad   | wheel 0, cumulative |
| 54–57  | `float`   | `pos1`       | rad   | wheel 1 |
| 58–61  | `float`   | `vel0`       | rad/s | wheel 0 |
| 62–65  | `float`   | `vel1`       | rad/s | wheel 1 |
| 66–69  | `float`   | `u0`         | V or A| command **applied**, not received |
| 70–73  | `float`   | `u1`         | V or A| |
| 74–77  | `float`   | `foc_hz`     | Hz    | measured FOC loop rate |
| 78–81  | `float`   | `imu_hz`     | Hz    | measured IMU link rate |
| 82–85  | `float`   | `imu_age_ms` | ms    | time since the last IMU frame |
| 86     | `uint8`   | `status`     | —     | see below |
| 87     | `uint8`   | `crc`        | —     | XOR over bytes 2…86 |

The full sensor vectors are forwarded, not just the selected pair. That is
what lets `s01_imu_axis.m` identify the balance axis from one tilting motion
instead of three, and what will later feed a complementary filter or a Kalman
observer without any firmware change.

`u0`/`u1` are what was **actually applied**, not what was received. The gap
between what you sent and what appears here is saturation or a safety
intervention — and it is the channel `s01` uses to measure round-trip latency.

`foc_hz` and `imu_hz` are measured, not assumed. They are the cheapest health
indicators in the system: a collapsing `foc_hz` means the I²C encoder reads are
choking, and an `imu_hz` below ~100 means the UART link is degrading.

### `status`

Bits 1 and 5 separate two failures that look identical from the outside.
Bit 5 says frames are arriving with a valid checksum; bit 1 says those frames
carry usable sensor data. `imu_ok = 0` with `imu_link = 1` is a sensor problem
on the IMU node — its I²C bus, almost always electrical noise from the motors.
`imu_ok = 0` with `imu_link = 0` is the UART between the boards, or the sensor
node's power. Without the distinction you cannot tell which cable to touch.

| Bits | Meaning |
|-----:|---------|
| 0    | motors armed |
| 1    | IMU frames are fresh |
| 2    | tilt latch active |
| 3    | PC watchdog latch active |
| 4    | IMU-link latch: the link dropped at some point while armed |
| 5    | IMU frames pass the CRC, whatever they contain |
| 6–7  | mode: `0` idle, `1` direct, `2` onboard PID |

---

## Link B · MATLAB → motor node · 16 bytes

| Offset | Type    | Field |
|-------:|---------|-------|
| 0–1    | `uint8` | `0xA5 0x5A` |
| 2      | `uint8` | type |
| 3–6    | `float` | `p1` |
| 7–10   | `float` | `p2` |
| 11–14  | `float` | `p3` |
| 15     | `uint8` | XOR over bytes 2…14 |

| Type   | Name       | `p1` | `p2` | `p3` |
|--------|------------|------|------|------|
| `0x01` | TORQUE     | motor 0 command | motor 1 command | — |
| `0x02` | GAINS      | `Kp` | `Ki` | `Kd` |
| `0x03` | MODE       | 0 idle / 1 direct / 2 PID | — | — |
| `0x04` | SETPOINT   | reference angle [rad] | — | — |
| `0x05` | LIMITS     | `u_max` | max tilt [rad] | — |
| `0x06` | ARM        | 1 arm (clears latches) / 0 disarm | — | — |
| `0x07` | TRIM       | equilibrium offset [rad] | — | — |
| `0x08` | IMU_AXIS   | angle selector | rate selector | — |

In firmware these identifiers are `SBR_TORQUE`, `SBR_GAINS`, … — not `CMD_*`.
SimpleFOC's `communication/commands.h` defines `CMD_LIMITS`, `CMD_STATUS` and
others as character macros, so an enum constant sharing one of those names is
expanded into `'L' = 0x05` before the compiler ever sees it. That whole
namespace belongs to SimpleFOC.

Both structs and the command ids live in `protocol.h`, never in the `.ino`.
Arduino hoists generated function prototypes above the sketch body, so a
function taking a struct declared later in the same file fails with
`'Cmd' does not name a type`. Types used in signatures must come from an
include.

**Channel selectors** are signed: `±(index+1)`, where `0` means *leave
unchanged*. So `+3` selects Euler index 2 with a `+` sign, and `-1` selects
index 0 with a `−` sign. Packing sign and index into one float lets the angle
and the rate carry independent signs — which they sometimes must, because the
Euler convention and the gyro convention are not guaranteed to agree on a
given axis.

---

## Behaviour

**PC watchdog.** Any valid command resets it. No traffic for `WDT_TIMEOUT_MS`
(250 ms) and the motors are cut with `wdt_fault` raised. This applies in
onboard-PID mode too — MATLAB must keep sending *something*, which is exactly
what `SbrLink.heartbeat()` is for.

**IMU staleness.** No valid IMU frame for `IMU_TIMEOUT_MS` (50 ms, about five
missed samples) and `imu_ok` drops, which cuts the motors immediately. This is
not latched — the wheels restart on their own once frames resume — but
`imu_fault` latches so you can see afterwards that it happened.

**Latches.** `tilt_fault`, `wdt_fault` and `imu_fault` persist. They clear
**only** on `ARM(1)`. That is deliberate: a robot that re-arms itself after
falling is a robot that hits the wall twice.

**Arming.** Motors boot disabled. `ARM(1)` is mandatory. `initFOC()` spins both
motors to find the electrical zero, so hold the robot off the ground at every
power-up.

**Backlog.** The motor node emits continuously at 50 Hz. If MATLAB runs
slower, frames pile up. `SbrLink.readLatest()` always returns the newest valid
frame and discards the rest — acting on stale data is worse than skipping a
sample. The `nDropped` counter shows how much was thrown away.

**Command units.** With `PHASE_RESISTANCE = 0` in `config.h`, commands are in
**volts**. Set it above zero and SimpleFOC computes `Uq = I·R`, so commands
become **amps** — an approximate torque interface. Switching rescales every
PID gain, so retune when you do.
