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

#define SERIAL_BAUD      115200
