// imu_node.ino - BNO055 sensor node. Samples at IMU_HZ and streams a
// binary frame over UART2 to the motor node. No control logic.
// UART is crossed (TX17 -> RX16) and both boards must share a ground.

#include <Arduino.h>
#include <Wire.h>
#include "protocol.h"

#define IMU_SDA        19
#define IMU_SCL        18
#define IMU_I2C_HZ     400000UL

#define BNO_ADDR       0x28

#define BNO_MODE       0x08

#define LINK_RX        16
#define LINK_TX        17
#define LINK_BAUD      460800

#define IMU_HZ         100

#define FAIL_TOLERANCE   5
#define FAIL_HARD_RESET  100

#define STATUS_LED     2

HardwareSerial Link(2);

static bool     g_ok         = false;

static bool     g_configured = false;
static uint8_t  g_calib     = 0;
static uint16_t g_fail_run  = 0;
static uint32_t g_fail_tot  = 0;
static uint32_t g_reset_ms  = 0;

static bool bnoWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool bnoRead(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)BNO_ADDR, (int)n, (int)true) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool bnoInit() {

  g_configured = false;

  uint8_t id = 0;
  if (!bnoRead(0x00, &id, 1)) return false;
  if (id != 0xA0) return false;

  if (!bnoWrite8(0x3D, 0x00)) return false;
  delay(25);
  bnoWrite8(0x3F, 0x20);
  delay(700);
  for (int i = 0; i < 20 && id != 0xA0; i++) { bnoRead(0x00, &id, 1); delay(10); }
  if (id != 0xA0) return false;

  bnoWrite8(0x3E, 0x00);
  delay(10);
  bnoWrite8(0x07, 0x00);

  bnoWrite8(0x3B, 0x00);
  bnoWrite8(0x3F, 0x00);
  delay(10);
  if (!bnoWrite8(0x3D, BNO_MODE)) return false;
  delay(30);

  uint8_t mode = 0xFF;
  if (!bnoRead(0x3D, &mode, 1)) return false;
  if ((mode & 0x0F) != BNO_MODE) return false;

  g_configured = true;
  return true;
}

static bool bnoReadAll(float eul[3], float gyr[3], float acc[3]) {
  uint8_t b[24];
  if (!bnoRead(0x08, b, 24)) return false;

  auto s16 = [&](int i) -> int16_t {
    return (int16_t)((uint16_t)b[i] | ((uint16_t)b[i + 1] << 8));
  };

  for (int i = 0; i < 3; i++) acc[i] = s16(0  + 2 * i) / 100.0f;
  for (int i = 0; i < 3; i++) gyr[i] = s16(12 + 2 * i) / 16.0f * DEG_TO_RAD;
  for (int i = 0; i < 3; i++) eul[i] = s16(18 + 2 * i) / 16.0f * DEG_TO_RAD;
  return true;
}

void setup() {
  Serial.begin(115200);
  if (STATUS_LED >= 0) pinMode(STATUS_LED, OUTPUT);

  Link.begin(LINK_BAUD, SERIAL_8N1, LINK_RX, LINK_TX);

  Wire.begin(IMU_SDA, IMU_SCL, IMU_I2C_HZ);
  Wire.setTimeOut(10);

  delay(50);
  g_ok = bnoInit();

  Serial.println(g_ok ? F("imu_node: BNO055 ready")
                      : F("imu_node: BNO055 NOT FOUND — retrying in background"));
}

void loop() {
  static uint32_t t_next   = 0;
  static uint32_t t_report = 0;
  static uint32_t n_sent   = 0;
  static float    eul[3]   = {0}, gyr[3] = {0}, acc[3] = {0};

  const uint32_t now = micros();
  if ((int32_t)(now - t_next) < 0) return;
  t_next = now + (1000000UL / IMU_HZ);

  if (bnoReadAll(eul, gyr, acc) && g_configured) {
    g_ok = true;
    g_fail_run = 0;
  } else {
    g_fail_tot++;
    if (g_fail_run < 0xFFFF) g_fail_run++;

    if (g_fail_run >= FAIL_TOLERANCE) {
      g_ok = false;
    }
    if (g_fail_run == FAIL_TOLERANCE) {
      Wire.end();
      Wire.begin(IMU_SDA, IMU_SCL, IMU_I2C_HZ);
      Wire.setTimeOut(10);
    }
    if (g_fail_run >= FAIL_HARD_RESET && millis() - g_reset_ms > 2000) {

      g_reset_ms = millis();
      g_ok = bnoInit();
      g_fail_run = 0;
    }
  }

  static uint8_t calib_div = 0;
  if (++calib_div >= IMU_HZ / 10) {
    calib_div = 0;
    uint8_t c;
    if (bnoRead(0x35, &c, 1)) g_calib = c;

    uint8_t mode = 0xFF;
    if (bnoRead(0x3D, &mode, 1) && (mode & 0x0F) != BNO_MODE) {
      g_configured = false;
      g_ok = false;
    }
  }

  if (!g_configured && millis() - g_reset_ms > 2000) {
    g_reset_ms = millis();
    bnoInit();
  }

  ImuPkt p;
  p.s0 = SBR_IMU_SYNC0; p.s1 = SBR_IMU_SYNC1;
  p.t_us = now;
  for (int i = 0; i < 3; i++) { p.eul[i] = eul[i]; p.gyr[i] = gyr[i]; p.acc[i] = acc[i]; }

  const uint8_t sys = (g_calib >> 6) & 0x03;
  const uint8_t gyc = (g_calib >> 4) & 0x03;
  const uint8_t acc_c = (g_calib >> 2) & 0x03;
  p.status = (g_ok ? 0x01 : 0x00) | (uint8_t)(sys << 2) | (uint8_t)(gyc << 4)
                                  | (uint8_t)(acc_c << 6);
  p.crc = sbrXor((uint8_t *)&p + 2, sizeof(ImuPkt) - 3);

  Link.write((uint8_t *)&p, sizeof(ImuPkt));
  n_sent++;

  if (millis() - t_report > 1000) {
    t_report = millis();
    if (STATUS_LED >= 0) digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));

    Serial.printf("ok=%d cfg=%d  %lu Hz  i2c_fail=%lu  eul=[%6.1f %6.1f %6.1f] deg  calib sys=%u gyr=%u\n",
                  (int)g_ok, (int)g_configured,
                  (unsigned long)n_sent, (unsigned long)g_fail_tot,
                  eul[0] * RAD_TO_DEG, eul[1] * RAD_TO_DEG, eul[2] * RAD_TO_DEG,
                  sys, gyc);
    n_sent = 0; g_fail_tot = 0;
  }
}
