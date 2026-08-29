// motor_node.ino - dual FOC, encoders, IMU link, safety, MATLAB protocol.
// ESP32 Dev Module + MKS DUAL FOC v3.2 Plus. Flash once; mode, gains,
// setpoint, limits and IMU axis all arrive over serial at runtime.
// See docs/protocol.md and docs/hardware.md.

#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include "config.h"
#include "protocol.h"

#if PC_LINK_WIFI
#include <WiFi.h>
static WiFiServer pcServer(WIFI_TCP_PORT);
static WiFiClient pcClient;
#endif

BLDCMotor         motor0(POLE_PAIRS);
BLDCMotor         motor1(POLE_PAIRS);
BLDCDriver3PWM    drv0(M0_A, M0_B, M0_C, M0_EN);
BLDCDriver3PWM    drv1(M1_A, M1_B, M1_C, M1_EN);
MagneticSensorI2C enc0(AS5600_I2C);
MagneticSensorI2C enc1(AS5600_I2C);

HardwareSerial ImuLink(2);

enum Mode : uint8_t { MODE_IDLE = 0, MODE_DIRECT = 1, MODE_PID = 2 };

static Mode  g_mode       = MODE_IDLE;
static bool  g_armed      = false;
static bool  g_imu_ok     = false;
static bool  g_imu_seen   = false;
static bool  g_imu_link   = false;
static bool  g_imu_fault  = false;
static bool  g_tilt_fault = false;
static bool  g_wdt_fault  = false;

static float g_u0 = 0.0f, g_u1 = 0.0f;
static float g_cmd0 = 0.0f, g_cmd1 = 0.0f;

static float g_kp = 0.0f, g_ki = 0.0f, g_kd = 0.0f;
static float g_integ = 0.0f;
static float g_ref_angle = 0.0f;
static float g_trim = 0.0f;
static float g_u_limit    = U_LIMIT_DEFAULT;
static float g_tilt_limit = TILT_LIMIT_RAD;

static float g_eul[3] = {0, 0, 0};
static float g_gyr[3] = {0, 0, 0};
static float g_acc[3] = {0, 0, 0};

static float g_angle = 0.0f;
static float g_rate  = 0.0f;

static uint8_t g_ang_idx  = IMU_ANGLE_IDX;
static float   g_ang_sign = IMU_ANGLE_SIGN;
static uint8_t g_gyr_idx  = IMU_GYRO_IDX;
static float   g_gyr_sign = IMU_GYRO_SIGN;

static bool     g_align_ok = false;
static float    g_foc_hz = 0.0f, g_imu_hz = 0.0f;
static uint32_t g_imu_last_us = 0;
static uint32_t g_imu_frame_us = 0;
static uint32_t g_last_cmd_ms = 0;

static uint8_t imuBuf[sizeof(ImuPkt)];
static uint8_t imuIdx = 0;
static uint32_t g_imu_count = 0;

static void applyImuSelection() {
  g_angle = g_ang_sign * g_eul[g_ang_idx];
  g_rate  = g_gyr_sign * g_gyr[g_gyr_idx];
}

static void serviceImuLink() {
  while (ImuLink.available()) {
    uint8_t b = ImuLink.read();

    if (imuIdx == 0) { if (b == SBR_IMU_SYNC0) imuBuf[imuIdx++] = b; continue; }
    if (imuIdx == 1) {
      if (b == SBR_IMU_SYNC1) imuBuf[imuIdx++] = b;
      else                    imuIdx = (b == SBR_IMU_SYNC0) ? 1 : 0;
      continue;
    }

    imuBuf[imuIdx++] = b;
    if (imuIdx < sizeof(ImuPkt)) continue;
    imuIdx = 0;

    ImuPkt p;
    memcpy(&p, imuBuf, sizeof(ImuPkt));
    if (sbrXor(imuBuf + 2, sizeof(ImuPkt) - 3) != p.crc) continue;
    g_imu_frame_us = micros();
    if (!(p.status & 0x01)) continue;

    for (int i = 0; i < 3; i++) {
      g_eul[i] = p.eul[i];
      g_gyr[i] = p.gyr[i];
      g_acc[i] = p.acc[i];
    }
    applyImuSelection();
    g_imu_last_us = micros();
    g_imu_seen = true;
    g_imu_count++;
  }
}

static uint8_t rxBuf[sizeof(CmdPkt)];
static uint8_t rxIdx = 0;

static bool decodeSel(float v, uint8_t &idx, float &sgn) {
  const int n = (int)lroundf(fabsf(v));
  if (n < 1 || n > 3) return false;
  idx = (uint8_t)(n - 1);
  sgn = (v < 0) ? -1.0f : 1.0f;
  return true;
}

static void applyCommand(const CmdPkt &c) {
  switch (c.type) {
    case SBR_TORQUE:
      g_cmd0 = c.p1; g_cmd1 = c.p2;
      break;
    case SBR_GAINS:
      g_kp = c.p1; g_ki = c.p2; g_kd = c.p3;
      g_integ = 0.0f;
      break;
    case SBR_MODE: {
      const int m = (int)lroundf(c.p1);
      if (m < 0 || m > 2) return;
      g_mode  = (Mode)m;
      g_integ = 0.0f;
      g_cmd0  = g_cmd1 = 0.0f;
      break;
    }
    case SBR_SETPOINT:
      g_ref_angle = c.p1;
      break;
    case SBR_LIMITS:
      if (c.p1 > 0) g_u_limit    = c.p1;
      if (c.p2 > 0) g_tilt_limit = c.p2;
      break;
    case SBR_ARM:
      if (c.p1 > 0.5f) {
        g_tilt_fault = false;
        g_wdt_fault  = false;
        g_imu_fault  = false;
        g_integ      = 0.0f;
        g_armed      = true;
      } else {
        g_armed = false;
      }

      break;
    case SBR_TRIM:
      g_trim = c.p1;
      break;
    case SBR_IMUAXIS:
      decodeSel(c.p1, g_ang_idx, g_ang_sign);
      decodeSel(c.p2, g_gyr_idx, g_gyr_sign);
      applyImuSelection();
      break;
    default:
      return;
  }
  g_last_cmd_ms = millis();
}

static Stream *pcLink() {
#if PC_LINK_WIFI
  return pcClient.connected() ? (Stream *)&pcClient : nullptr;
#else
  return (Stream *)&Serial;
#endif
}

static void pcWrite(const uint8_t *b, size_t n) {
#if PC_LINK_WIFI
  if (!pcClient.connected()) return;
  if ((size_t)pcClient.availableForWrite() < n) return;
  pcClient.write(b, n);
#else
  Serial.write(b, n);
#endif
}

static void servicePcTransport() {
#if PC_LINK_WIFI
  if (pcClient.connected()) {
    if (!pcServer.hasClient()) return;
    pcServer.accept().stop();
    return;
  }
  if (pcClient) pcClient.stop();
  if (pcServer.hasClient()) {
    pcClient = pcServer.accept();
    pcClient.setNoDelay(true);
    rxIdx = 0;
  }
#endif
}

static void servicePcLink() {
  Stream *pc = pcLink();
  if (pc == nullptr) { rxIdx = 0; return; }

  while (pc->available()) {
    uint8_t b = pc->read();

    if (rxIdx == 0) { if (b == SBR_PC_SYNC0) rxBuf[rxIdx++] = b; continue; }
    if (rxIdx == 1) {
      if (b == SBR_PC_SYNC1) rxBuf[rxIdx++] = b;
      else                   rxIdx = (b == SBR_PC_SYNC0) ? 1 : 0;
      continue;
    }

    rxBuf[rxIdx++] = b;
    if (rxIdx < sizeof(CmdPkt)) continue;
    rxIdx = 0;

    CmdPkt c;
    memcpy(&c, rxBuf, sizeof(CmdPkt));
    if (sbrXor(rxBuf + 2, sizeof(CmdPkt) - 3) == c.crc) applyCommand(c);
  }
}

static void sendTelemetry(float imu_age_ms) {
  TelemPkt t;
  t.s0 = SBR_PC_SYNC0; t.s1 = SBR_PC_SYNC1;
  t.t_us = micros();
  for (int i = 0; i < 3; i++) {
    t.eul[i] = g_eul[i]; t.gyr[i] = g_gyr[i]; t.acc[i] = g_acc[i];
  }
  t.angle = g_angle;  t.rate = g_rate;
  t.pos0  = motor0.shaft_angle;     t.pos1 = motor1.shaft_angle;
  t.vel0  = motor0.shaft_velocity;  t.vel1 = motor1.shaft_velocity;
  t.u0    = g_u0;  t.u1 = g_u1;

  t.foc_hz = g_align_ok ? g_foc_hz : -1.0f;
  t.imu_hz = g_imu_hz;
  t.imu_age_ms = imu_age_ms;
  t.status = (g_armed      ? 0x01 : 0) |
             (g_imu_ok     ? 0x02 : 0) |
             (g_tilt_fault ? 0x04 : 0) |
             (g_wdt_fault  ? 0x08 : 0) |
             (g_imu_fault  ? 0x10 : 0) |
             (g_imu_link   ? 0x20 : 0) |
             ((uint8_t)g_mode << 6);
  t.crc = sbrXor((uint8_t *)&t + 2, sizeof(TelemPkt) - 3);
  pcWrite((uint8_t *)&t, sizeof(TelemPkt));
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  ImuLink.begin(IMU_BAUD, SERIAL_8N1, IMU_RX, IMU_TX);
  delay(300);

#if PC_LINK_WIFI
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL);
  WiFi.setSleep(false);
  pcServer.begin();
  pcServer.setNoDelay(true);
  Serial.print(F("AP \"" WIFI_AP_SSID "\" up, connect to "));
  Serial.print(WiFi.softAPIP());
  Serial.print(F(" port "));
  Serial.println(WIFI_TCP_PORT);
#endif

  Wire.begin(I2C0_SDA, I2C0_SCL, I2C_ENC_HZ);
  Wire1.begin(I2C1_SDA, I2C1_SCL, I2C_ENC_HZ);
  Wire.setTimeOut(10);
  Wire1.setTimeOut(10);

  enc0.init(M0_ENC_BUS == 0 ? &Wire : &Wire1);
  enc1.init(M1_ENC_BUS == 0 ? &Wire : &Wire1);
  motor0.linkSensor(&enc0);
  motor1.linkSensor(&enc1);

  drv0.voltage_power_supply = V_SUPPLY;
  drv1.voltage_power_supply = V_SUPPLY;
  drv0.init();
  drv1.init();
  motor0.linkDriver(&drv0);
  motor1.linkDriver(&drv1);

  for (BLDCMotor *m : { &motor0, &motor1 }) {
    m->foc_modulation    = FOCModulationType::SpaceVectorPWM;
    m->controller        = MotionControlType::torque;
    m->torque_controller = TorqueControlType::voltage;
    m->voltage_limit     = V_LIMIT;
    if (PHASE_RESISTANCE > 0.0f) {
      m->phase_resistance = PHASE_RESISTANCE;
      m->current_limit    = CURRENT_LIMIT;
      if (MOTOR_KV > 0.0f) m->KV_rating = MOTOR_KV;
    }
  }

  motor0.init();  const int ok0 = motor0.initFOC();
  motor1.init();  const int ok1 = motor1.initFOC();
  g_align_ok = (ok0 == 1) && (ok1 == 1);

  if (!g_align_ok) {

    Serial.println();
    Serial.print(F("FOC ALIGNMENT FAILED  motor0=")); Serial.print(ok0);
    Serial.print(F("  motor1="));                     Serial.println(ok1);
    Serial.println(F("Check: magnet centred over each AS5600, 12 V supply"));
    Serial.println(F("live at power-up, M0_ENC_BUS/M1_ENC_BUS in config.h."));
  }

  motor0.disable();
  motor1.disable();

  g_last_cmd_ms = millis();
  g_imu_last_us = micros();
  g_imu_frame_us = micros();
}

void loop() {
  static uint32_t t_next_ctrl = 0;
  static uint32_t foc_count   = 0;
  static uint32_t t_hz_mark   = 0;
  static uint8_t  telem_div   = 0;

  motor0.loopFOC();
  motor1.loopFOC();
  foc_count++;

  serviceImuLink();
  servicePcTransport();
  servicePcLink();

  const uint32_t now_us = micros();
  if ((int32_t)(now_us - t_next_ctrl) < 0) return;
  t_next_ctrl = now_us + (1000000UL / CTRL_HZ);

  static bool hz_primed = false;
  if (!hz_primed) {
    t_hz_mark = now_us;
    foc_count = 0; g_imu_count = 0;
    hz_primed = true;
  } else if (now_us - t_hz_mark >= 500000UL) {
    const float win = (float)(now_us - t_hz_mark);
    g_foc_hz = foc_count   * 1e6f / win;
    g_imu_hz = g_imu_count * 1e6f / win;
    foc_count = 0; g_imu_count = 0; t_hz_mark = now_us;
  }

  const float imu_age_ms = (float)(uint32_t)(now_us - g_imu_last_us) / 1000.0f;
  g_imu_ok = g_imu_seen && imu_age_ms < (float)IMU_TIMEOUT_MS;
  g_imu_link = (float)(uint32_t)(now_us - g_imu_frame_us) / 1000.0f
               < (float)IMU_TIMEOUT_MS;

  if (!g_imu_ok && g_armed)                      g_imu_fault  = true;
  if (millis() - g_last_cmd_ms > WDT_TIMEOUT_MS) g_wdt_fault  = true;
  if (fabsf(g_angle - g_trim) > g_tilt_limit)    g_tilt_fault = true;

  const bool safe = g_align_ok && g_armed && g_imu_ok && !g_tilt_fault && !g_wdt_fault;

  static bool prev_safe = false;
  if (safe != prev_safe) {
    if (safe) { motor0.enable();  motor1.enable();  }
    else      { motor0.disable(); motor1.disable(); }
    prev_safe = safe;
  }

  if (!safe) {
    g_u0 = g_u1 = 0.0f;
    g_integ = 0.0f;
  } else {
    if (g_mode == MODE_DIRECT) {
      g_u0 = constrain(g_cmd0, -g_u_limit, g_u_limit);
      g_u1 = constrain(g_cmd1, -g_u_limit, g_u_limit);
    } else if (g_mode == MODE_PID) {
      const float dt  = 1.0f / CTRL_HZ;
      const float err = (g_ref_angle + g_trim) - g_angle;

      g_integ += g_ki * err * dt;
      g_integ  = constrain(g_integ, -g_u_limit, g_u_limit);

      float u = g_kp * err + g_integ - g_kd * g_rate;
      u = constrain(u, -g_u_limit, g_u_limit);

      g_u0 = u;
      g_u1 = u;
    } else {
      g_u0 = g_u1 = 0.0f;
    }
  }

  motor0.move(MOTOR0_DIR * g_u0);
  motor1.move(MOTOR1_DIR * g_u1);

  if (++telem_div >= TELEM_DIV) {
    telem_div = 0;
    sendTelemetry(imu_age_ms);
  }
}
