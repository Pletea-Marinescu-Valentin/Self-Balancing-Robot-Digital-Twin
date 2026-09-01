#pragma once

// protocol.h - wire format shared by both nodes. IDENTICAL COPY in
// firmware/imu_node/; matlab/test_protocol.m fails if they drift apart.
// Arduino cannot include across sketch folders, hence the duplication.
// SBR_ prefix, not CMD_: SimpleFOC owns the CMD_* macro namespace.

#include <Arduino.h>

#define SBR_IMU_SYNC0  0xA7
#define SBR_IMU_SYNC1  0x5B

struct __attribute__((packed)) ImuPkt {
  uint8_t  s0, s1;
  uint32_t t_us;
  float    eul[3];
  float    gyr[3];
  float    acc[3];
  uint8_t  status;
  uint8_t  crc;
};
static_assert(sizeof(ImuPkt) == 44, "ImuPkt must be 44 bytes - check packing");

#define SBR_PC_SYNC0   0xA5
#define SBR_PC_SYNC1   0x5A

// ref / vel_fwd / pos_fwd are the outer (wheel) loop's own state. They are on
// the wire because that loop cannot be tuned from the angle trace alone: the
// balance angle can look perfect while the robot rolls off the table.
//   ref      commanded lean [rad], the outer loop's output
//   vel_fwd  filtered forward wheel speed [rad/s], the outer loop's input
//   pos_fwd  forward wheel travel [rad] since the robot last caught itself
struct __attribute__((packed)) TelemPkt {
  uint8_t  s0, s1;
  uint32_t t_us;
  float    eul[3];
  float    gyr[3];
  float    acc[3];
  float    angle, rate;
  float    pos0, pos1, vel0, vel1;
  float    u0, u1;
  float    foc_hz, imu_hz, imu_age_ms;
  float    ref, vel_fwd, pos_fwd;
  uint8_t  status;
  uint8_t  crc;
};
static_assert(sizeof(TelemPkt) == 100, "TelemPkt must be 100 bytes - check packing");

struct __attribute__((packed)) CmdPkt {
  uint8_t s0, s1, type;
  float   p1, p2, p3;
  uint8_t crc;
};
static_assert(sizeof(CmdPkt) == 16, "CmdPkt must be 16 bytes - check packing");

enum SbrCmd : uint8_t {
  SBR_TORQUE   = 0x01,
  SBR_GAINS    = 0x02,
  SBR_MODE     = 0x03,
  SBR_SETPOINT = 0x04,
  SBR_LIMITS   = 0x05,
  SBR_ARM      = 0x06,
  SBR_TRIM     = 0x07,
  SBR_IMUAXIS  = 0x08,
  SBR_OUTER    = 0x09,
  SBR_YAW      = 0x0A,
  SBR_FRICTION = 0x0B
};

static inline uint8_t sbrXor(const uint8_t *p, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) c ^= p[i];
  return c;
}
