#pragma once

// config.h - every tunable for the motor node. Change hardware, change
// this file, nothing else. Values are explained in docs/hardware.md.

#define M0_A        32
#define M0_B        33
#define M0_C        25
#define M0_EN       22

#define M1_A        26
#define M1_B        27
#define M1_C        14
#define M1_EN       12

#define I2C0_SDA    19
#define I2C0_SCL    18
#define I2C1_SDA    23
#define I2C1_SCL     5
#define I2C_ENC_HZ  400000UL

#define M0_ENC_BUS  1
#define M1_ENC_BUS  0

#define IMU_RX          16
#define IMU_TX          17
#define IMU_BAUD        460800
#define IMU_TIMEOUT_MS  50

#define POLE_PAIRS       7

#define PHASE_RESISTANCE 0.0f

#define MOTOR_KV         0.0f

#define V_SUPPLY         10.0f
#define V_LIMIT           8.0f
#define CURRENT_LIMIT     0.8f

#define MOTOR0_DIR      (+1.0f)
#define MOTOR1_DIR      (-1.0f)

// Measured on this build, not a guess: held upright, pitch (index 2) read a
// constant 0.0000 rad while roll tracked the tilt. s01_imu_axis.m re-measures
// both and pushes them at runtime, so these are only the power-on defaults.
#define IMU_ANGLE_IDX    2
#define IMU_ANGLE_SIGN (+1.0f)
#define IMU_GYRO_IDX     0
#define IMU_GYRO_SIGN  (+1.0f)

#define CTRL_HZ          200

#define TELEM_DIV        4

#define PC_LINK_WIFI     1
#define WIFI_AP_SSID     "sbr-robot"
#define WIFI_AP_PASS     "balancebot"
#define WIFI_AP_CHANNEL  6
#define WIFI_TCP_PORT    3333

#if PC_LINK_WIFI
#define WDT_TIMEOUT_MS   1000
#else
#define WDT_TIMEOUT_MS   250
#endif
#define TILT_LIMIT_RAD   0.60f
#define U_LIMIT_DEFAULT   3.0f

// ---------------------------------------------------------------------------
// Outer loop - wheel speed and travel
//
// The balance PID alone regulates one state, the tilt angle. Wheel speed and
// wheel travel never enter it, so they are not controlled at all: a robot
// standing perfectly upright while crossing the room is, to that loop, a
// perfect solution. The result is the "balances fine, drives off the table"
// behaviour. The fix is a second, slower loop that feeds the balance loop a
// lean to aim at, because leaning back is the only way a wheeled inverted
// pendulum can slow down.
//
// Rate ratio follows the usual cascade practice of roughly 5:1 - the outer
// loop must be far slower than the inner one or the two fight each other.
#define OUTER_DIV         5              // outer loop at CTRL_HZ / OUTER_DIV
#define VEL_LPF_HZ        2.0f           // wheel-speed filter for the outer loop
#define FOC_VEL_TF        0.005f         // SimpleFOC's own velocity filter [s]

// Gains are in radians of lean per unit of wheel state. Signs are positive by
// construction: forward wheel state is built with the same MOTORn_DIR used to
// drive, and initFOC() aligns each encoder so +u gives +shaft_velocity.
#define KV_DEFAULT        0.010f         // rad of lean per (rad/s) of wheel speed
#define KVI_DEFAULT       0.0026f        // rad of lean per rad of wheel travel
#define REF_LIMIT_DEFAULT 0.052f         // 3 deg. The outer loop's whole authority.

// Yaw hold, from the wheel encoders alone - no extra sensor. Differential
// wheel travel is a heading proxy, and damping it stops the slow left-right
// wander. Kp defaults off: damping is safe, holding a heading is a preference.
#define KYAW_P_DEFAULT    0.0f
#define KYAW_D_DEFAULT    0.05f

// Coulomb friction feedforward. Gimbal motors in torque mode have real
// stiction near zero command, which shows up as hunting around the setpoint.
// Off by default - it is a fix for a specific symptom, not a freebie.
#define U_FRIC_DEFAULT    0.0f
#define V_FRIC_EPS        0.5f           // rad/s ramp width, avoids chattering

#define SERIAL_BAUD      115200
