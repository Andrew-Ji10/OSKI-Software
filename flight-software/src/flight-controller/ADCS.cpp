#include "ADCS.h"
#include "CAM.h"
#include <EEPROM.h>

#define UART_TX 1
#define UART_RX 2

namespace ADCS {

// ==========================
// BNO055 CONFIG
// ==========================

static constexpr int BNO_SDA = 8;
static constexpr int BNO_SCL = 7;
static constexpr uint8_t BNO_ADDR = 0x28;

static Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDR, &Wire);

// Change this later to Serial1 / UART to protoboard
#define WHEEL_SERIAL Serial1

// ==========================
// CONTROL CONSTANTS
// ==========================

static constexpr float JW_Z = 6.0e-6f;

static constexpr float TAU_MAX = 3.11e-4f;
static constexpr float RAMP_MAX = 100.0f;

static float KP_Z = 2.0e-4f;
static float KD_Z = 2.0e-4f;
static float KI_Z = 0.0f;

static constexpr float GYRO_DEADBAND = 0.20f;
static constexpr float ANGLE_ERR_DEADBAND = 0.06f;
static constexpr float GYRO_FILTER_ALPHA = 0.25f;
static constexpr float DETUMBLE_RATE_ENTER = 80.00f;
static constexpr float DETUMBLE_RATE_EXIT = 10.0f;

// Integrator windup clamp [rad*s] for angle error.
static constexpr float I_MAX = 0.1f;

static float int_z = 0.0f;

// Last measured attitude + gyro (updated each ADCS tick, read by telem task)
static float last_qw = 1.0f, last_qx = 0.0f, last_qy = 0.0f, last_qz = 0.0f;
static float last_wz = 0.0f;
static float filt_wz = 0.0f;
static float last_tau_z = 0.0f;
static float last_ramp_z = 0.0f;
static float last_err_z = 0.0f;
static bool detumbling = false;

static constexpr uint32_t ADCS_PERIOD_US = 20000;

// ==========================
// DESIRED ATTITUDE
// ==========================

static float q_des_w = 1.0f;
static float q_des_x = 0.0f;
static float q_des_y = 0.0f;
static float q_des_z = 0.0f;

static bool z_enabled = false;

// ==========================
// HELPERS
// ==========================

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float lowpass(float previous, float sample) {
  return previous + GYRO_FILTER_ALPHA * (sample - previous);
}

static bool shouldAcceptIntegratorStep(float tauBeforeStep, float deltaTauFromStep) {
  if (fabs(deltaTauFromStep) < 1.0e-12f) return true;
  if (fabs(tauBeforeStep) < TAU_MAX) return true;
  return (tauBeforeStep > 0.0f && deltaTauFromStep < 0.0f) ||
         (tauBeforeStep < 0.0f && deltaTauFromStep > 0.0f);
}

static void updateDetumbleState(float wz) {
  float rate = z_enabled ? fabs(wz) : 0.0f;
  if (detumbling) {
    detumbling = rate > DETUMBLE_RATE_EXIT;
  } else {
    detumbling = rate > DETUMBLE_RATE_ENTER;
  }
}

static void updateRateDampingController(float rate) {
  int_z = 0.0f;
  if (!z_enabled) {
    last_tau_z = 0.0f;
    last_ramp_z = 0.0f;
    return;
  }

  last_tau_z = clampf(-KD_Z * rate, -TAU_MAX, TAU_MAX);
  last_ramp_z = clampf((-last_tau_z) / JW_Z, -RAMP_MAX, RAMP_MAX);
}

static void updateYawController(float err, float rate) {
  if (!z_enabled) {
    int_z = 0.0f;
    last_tau_z = 0.0f;
    last_ramp_z = 0.0f;
    return;
  }

  static constexpr float DT = ADCS_PERIOD_US * 1.0e-6f;
  float baseTau = -KP_Z * err - KD_Z * rate;
  float tauBeforeStep = baseTau - KI_Z * int_z;
  float candidateInt = clampf(int_z + err * DT, -I_MAX, I_MAX);
  float deltaTauFromStep = -KI_Z * (candidateInt - int_z);

  if (shouldAcceptIntegratorStep(tauBeforeStep, deltaTauFromStep)) {
    int_z = candidateInt;
  }

  last_tau_z = clampf(baseTau - KI_Z * int_z, -TAU_MAX, TAU_MAX);
  last_ramp_z = clampf((-last_tau_z) / JW_Z, -RAMP_MAX, RAMP_MAX);
}

static void normalizeQuat(float &w, float &x, float &y, float &z) {
  float n = sqrtf(w*w + x*x + y*y + z*z);

  if (n < 1e-6f) {
    w = 1.0f;
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    return;
  }

  w /= n;
  x /= n;
  y /= n;
  z /= n;
}

static void quatConj(float w, float x, float y, float z,
                     float &ow, float &ox, float &oy, float &oz) {
  ow = w;
  ox = -x;
  oy = -y;
  oz = -z;
}

static void quatMultiply(float aw, float ax, float ay, float az,
                         float bw, float bx, float by, float bz,
                         float &ow, float &ox, float &oy, float &oz) {
  ow = aw*bw - ax*bx - ay*by - az*bz;
  ox = aw*bx + ax*bw + ay*bz - az*by;
  oy = aw*by - ax*bz + ay*bw + az*bx;
  oz = aw*bz + ax*by - ay*bx + az*bw;
}

// ==========================
// PACKET CALLBACKS
// ==========================

void setDesiredAttitude(Packet packet) {
  q_des_w = RadioComms::packetGetFloat(&packet, 0);
  q_des_x = RadioComms::packetGetFloat(&packet, 4);
  q_des_y = RadioComms::packetGetFloat(&packet, 8);
  q_des_z = RadioComms::packetGetFloat(&packet, 12);

  normalizeQuat(q_des_w, q_des_x, q_des_y, q_des_z);

  int_z = 0.0f;
  filt_wz = 0.0f;
  detumbling = false;
  Packet response;
  response.id = CMD_ADCS_SETPOINT;
  response.length = 0;
  RadioComms::emitPacket(&response);
}

void enableADCS(Packet packet) {
  z_enabled = packet.data[2] != 0;

  if (!z_enabled) { int_z = 0.0f; filt_wz = 0.0f; last_tau_z = 0.0f; last_ramp_z = 0.0f; last_err_z = 0.0f; }
  if (!z_enabled) detumbling = false;

  Packet response;
  response.id = CMD_ADCS_ENABLE;
  response.length = 0;

  RadioComms::packetAddUint8(&response, 0);
  RadioComms::packetAddUint8(&response, 0);
  RadioComms::packetAddUint8(&response, z_enabled ? 1 : 0);

  RadioComms::emitPacket(&response);
}

// ==========================
// WHEEL COMMANDS
// ==========================

// Block until a newline-terminated line arrives from WHEEL_SERIAL or timeoutMs elapses.
static bool readWheelAck(char* buf, size_t maxLen, uint32_t timeoutMs = 150) {
  uint32_t start = millis();
  size_t idx = 0;
  while (millis() - start < timeoutMs) {
    if (WHEEL_SERIAL.available()) {
      char c = (char)WHEEL_SERIAL.read();
      if (c == '\n') { buf[idx] = '\0'; return idx > 0; }
      if (c != '\r' && idx < maxLen - 1) buf[idx++] = c;
    }
  }
  buf[idx] = '\0';
  return false;
}

static void flushWheelRx() {
  while (WHEEL_SERIAL.available()) WHEEL_SERIAL.read();
}

static void packetAddStr(Packet* p, const char* s) {
  uint8_t len = (uint8_t)strnlen(s, sizeof(p->data) - p->length);
  memcpy(p->data + p->length, s, len);
  p->length += len;
}

static void sendADCSCommand(float ramp_z) {
  if (z_enabled) {
    WHEEL_SERIAL.print("YR");
    WHEEL_SERIAL.println(-ramp_z, 3);
  }
}

static void logADCSCommand() {
  static uint32_t lastLogMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastLogMs < 1000) return;

  lastLogMs = nowMs;
  Serial.printf("ADCS %s z_err=%.4f wz=%.4f tau=%.7f ramp=%.3f\n",
                detumbling ? "detumble" : "track",
                last_err_z, last_wz, last_tau_z, last_ramp_z);
}

static void zeroWheels() {
  WHEEL_SERIAL.println("YR0.000");
  WHEEL_SERIAL.println("YV0");
}

// CMD_ADCS_SET_PID payload: axis (uint8, only 2=Z is used) | Kp (float) | Ki (float) | Kd (float)
void setPIDGains(Packet packet) {
  uint8_t axis = RadioComms::packetGetUint8(&packet, 0);
  float kp     = RadioComms::packetGetFloat(&packet, 1);
  float ki     = RadioComms::packetGetFloat(&packet, 5);
  float kd     = RadioComms::packetGetFloat(&packet, 9);

  if (axis == 2) {
    KP_Z = kp;
    KI_Z = ki;
    KD_Z = kd;
    int_z = 0.0f;
  }

  Serial.printf("Yaw PID axis %u: Kp=%.6f Ki=%.6f Kd=%.6f\n", axis, KP_Z, KI_Z, KD_Z);

  Packet response;
  response.id = CMD_ADCS_SET_PID;
  response.length = 0;
  RadioComms::packetAddUint8(&response, 2);
  RadioComms::packetAddFloat(&response, KP_Z);
  RadioComms::packetAddFloat(&response, KI_Z);
  RadioComms::packetAddFloat(&response, KD_Z);
  RadioComms::emitPacket(&response);
}

// CMD_ADCS_WHEEL_VEL payload: axis (uint8, only 2=Z is used) | velocity (float)
void setWheelVelocity(Packet packet) {
  uint8_t axis = RadioComms::packetGetUint8(&packet, 0);
  float vel    = RadioComms::packetGetFloat(&packet, 1);

  if (axis != 2) return;

  z_enabled = false;
  int_z = 0.0f; filt_wz = 0.0f; last_tau_z = 0.0f; last_ramp_z = 0.0f; last_err_z = 0.0f;
  detumbling = false;
  WHEEL_SERIAL.print("YV");
  WHEEL_SERIAL.println(vel, 3);

  char ack[32];
  readWheelAck(ack, sizeof(ack));

  Packet response;
  response.id = CMD_ADCS_WHEEL_VEL;
  response.length = 0;
  RadioComms::packetAddUint8(&response, 2);
  RadioComms::packetAddFloat(&response, vel);
  packetAddStr(&response, ack);
  RadioComms::emitPacket(&response);
}

void zeroWheelsCmd(Packet packet) {
  z_enabled = false;

  int_z = 0.0f;
  filt_wz = 0.0f;
  last_tau_z = 0.0f;
  last_ramp_z = 0.0f;
  last_err_z = 0.0f;
  detumbling = false;
  zeroWheels();

  // zeroWheels() sends YR and YV which get echoed; collect both
  char ack1[32] = "", ack2[32] = "";
  readWheelAck(ack1, sizeof(ack1), 200);
  readWheelAck(ack2, sizeof(ack2), 200);

  Packet response;
  response.id = CMD_ADCS_ZERO;
  response.length = 0;
  packetAddStr(&response, ack1);
  if (ack2[0] != '\0') {
    RadioComms::packetAddUint8(&response, ' ');
    packetAddStr(&response, ack2);
  }
  RadioComms::emitPacket(&response);
}

void pingADCS(Packet packet) {
  WHEEL_SERIAL.println("P");

  char ack[32];
  bool alive = readWheelAck(ack, sizeof(ack));

  Packet response;
  response.id = CMD_ADCS_PING;
  response.length = 0;
  RadioComms::packetAddUint8(&response, alive ? 1 : 0);
  RadioComms::emitPacket(&response);
}

void powerOnADCS(Packet packet) {
  WHEEL_SERIAL.println("E");

  char ack[32];
  readWheelAck(ack, sizeof(ack));

  Packet response;
  response.id = CMD_ADCS_POWER_ON;
  response.length = 0;
  packetAddStr(&response, ack);
  RadioComms::emitPacket(&response);
}

void powerOffADCS(Packet packet) {
  WHEEL_SERIAL.println("D");

  char ack[32];
  readWheelAck(ack, sizeof(ack));

  Packet response;
  response.id = CMD_ADCS_POWER_OFF;
  response.length = 0;
  packetAddStr(&response, ack);
  RadioComms::emitPacket(&response);
}

// ==========================
// EEPROM CALIBRATION PERSISTENCE
// ==========================

static constexpr uint16_t EEPROM_MAGIC    = 0xCA1C;
static constexpr int      EEPROM_MAGIC_ADDR = 0;
static constexpr int      EEPROM_CAL_ADDR   = 2;
static constexpr int      EEPROM_SIZE = EEPROM_CAL_ADDR + sizeof(adafruit_bno055_offsets_t);

static void saveCalibrationToEEPROM() {
  adafruit_bno055_offsets_t offsets;
  bno.getSensorOffsets(offsets);
  EEPROM.put(EEPROM_CAL_ADDR, offsets);
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.commit();
  Serial.println("BNO055 calibration saved to EEPROM.");
}

static bool loadCalibrationFromEEPROM() {
  uint16_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  if (magic != EEPROM_MAGIC) {
    Serial.println("No valid BNO055 calibration in EEPROM.");
    return false;
  }
  adafruit_bno055_offsets_t offsets;
  EEPROM.get(EEPROM_CAL_ADDR, offsets);
  bno.setSensorOffsets(offsets);
  Serial.println("BNO055 calibration restored from EEPROM.");
  return true;
}

// ==========================
// CALIBRATION
// ==========================

static void printCalibrationStatus() {
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  Serial.print("CAL SYS/G/A/M = ");
  Serial.print(sys);
  Serial.print("/");
  Serial.print(gyro);
  Serial.print("/");
  Serial.print(accel);
  Serial.print("/");
  Serial.println(mag);
}

static void waitForBNOCalibration() {
  Serial.println("BNO055 calibration starting...");
  Serial.println("Keep still first for gyro, then slowly rotate/figure-8 for mag.");
  Serial.println("Target: sys=3, gyro=3, accel=3, mag=3.");

  const uint32_t timeout_ms = 300000;
  uint32_t start_ms = millis();

  while (millis() - start_ms < timeout_ms) {
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    Serial.print("CAL SYS/G/A/M = ");
    Serial.print(sys);
    Serial.print("/");
    Serial.print(gyro);
    Serial.print("/");
    Serial.print(accel);
    Serial.print("/");
    Serial.println(mag);

    if (sys >= 3 && gyro >= 3 && accel >= 3 && mag >= 3) {
      Serial.println("BNO055 calibration acceptable.");
      return;
    }

    delay(500);
  }

  Serial.println("BNO055 calibration timeout. Continuing anyway.");
  printCalibrationStatus();
}

// ==========================
// TARGET SET
// ==========================

static void setCurrentAttitudeAsTarget() {
  Serial.println("ADCS setting current attitude as target...");

  for (int i = 0; i < 20; i++) {
    imu::Quaternion q = bno.getQuat();

    q_des_w = q.w();
    q_des_x = q.x();
    q_des_y = q.y();
    q_des_z = q.z();

    normalizeQuat(q_des_w, q_des_x, q_des_y, q_des_z);

    Serial.print("ADCS q_des attempt ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(q_des_w, 4); Serial.print(",");
    Serial.print(q_des_x, 4); Serial.print(",");
    Serial.print(q_des_y, 4); Serial.print(",");
    Serial.println(q_des_z, 4);

    if (fabs(q_des_x) > 0.001f ||
        fabs(q_des_y) > 0.001f ||
        fabs(q_des_z) > 0.001f) {
      break;
    }

    delay(100);
  }

  Serial.print("ADCS target set q_des=");
  Serial.print(q_des_w, 4); Serial.print(",");
  Serial.print(q_des_x, 4); Serial.print(",");
  Serial.print(q_des_y, 4); Serial.print(",");
  Serial.println(q_des_z, 4);
}

// ==========================
// INIT
// ==========================

void init() {
  Serial.println("ADCS INIT ENTERED");
  Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  Wire.begin(BNO_SDA, BNO_SCL);
  Wire.setClock(100000);

  if (!bno.begin()) {
    Serial.println("ADCS ERR: BNO055 not found");
    return;
  }

  delay(500);
  bno.setExtCrystalUse(true);
  delay(500);

  EEPROM.begin(EEPROM_SIZE);
  if (!loadCalibrationFromEEPROM()) {
    waitForBNOCalibration();
    saveCalibrationToEEPROM();
  } else {
    printCalibrationStatus();
  }

  delay(300);
  setCurrentAttitudeAsTarget();

  RadioComms::registerCallback(CMD_ADCS_SETPOINT, setDesiredAttitude);
  RadioComms::registerCallback(CMD_ADCS_ENABLE, enableADCS);
  RadioComms::registerCallback(CMD_ADCS_ZERO, zeroWheelsCmd);
  RadioComms::registerCallback(CMD_ADCS_SET_PID, setPIDGains);
  RadioComms::registerCallback(CMD_ADCS_WHEEL_VEL, setWheelVelocity);
  RadioComms::registerCallback(CMD_ADCS_POWER_ON, powerOnADCS);
  RadioComms::registerCallback(CMD_ADCS_POWER_OFF, powerOffADCS);
  RadioComms::registerCallback(CMD_ADCS_PING, pingADCS);

  Serial.println("ADCS initialized");
}

// ==========================
// MAIN ADCS TASK
// ==========================
uint32_t task_runADCS() {
  imu::Quaternion q = bno.getQuat();
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  float qw = q.w();
  float qx = q.x();
  float qy = q.y();
  float qz = q.z();
  normalizeQuat(qw, qx, qy, qz);

  float wz = gyro.z();

  filt_wz = z_enabled ? lowpass(filt_wz, wz) : 0.0f;
  wz = filt_wz;

  if (!z_enabled || fabs(wz) < GYRO_DEADBAND) wz = 0.0f;

  last_qw = qw; last_qx = qx; last_qy = qy; last_qz = qz;
  last_wz = wz;

  updateDetumbleState(wz);

  if (detumbling) {
    last_err_z = 0.0f;
    updateRateDampingController(wz);

    if (CAM::isTransmitting()) {
      last_ramp_z = 0.0f;
      sendADCSCommand(0.0f);
    } else {
      sendADCSCommand(last_ramp_z);
    }

    logADCSCommand();
    return ADCS_PERIOD_US;
  }

  float qdw = q_des_w;
  float qdx = q_des_x;
  float qdy = q_des_y;
  float qdz = q_des_z;
  normalizeQuat(qdw, qdx, qdy, qdz);

  float qdes_inv_w, qdes_inv_x, qdes_inv_y, qdes_inv_z;
  quatConj(qdw, qdx, qdy, qdz,
           qdes_inv_w, qdes_inv_x, qdes_inv_y, qdes_inv_z);

  float qerr_w, qerr_x, qerr_y, qerr_z;
  quatMultiply(qdes_inv_w, qdes_inv_x, qdes_inv_y, qdes_inv_z,
               qw, qx, qy, qz,
               qerr_w, qerr_x, qerr_y, qerr_z);

  normalizeQuat(qerr_w, qerr_x, qerr_y, qerr_z);

  if (qerr_w < 0.0f) {
    qerr_w = -qerr_w;
    qerr_x = -qerr_x;
    qerr_y = -qerr_y;
    qerr_z = -qerr_z;
  }

  float qerr_vec_norm = sqrtf(qerr_x*qerr_x + qerr_y*qerr_y + qerr_z*qerr_z);
  float err_z = 0.0f;

  if (qerr_vec_norm > 1.0e-6f) {
    float angle = 2.0f * atan2f(qerr_vec_norm, qerr_w);
    float scale = angle / qerr_vec_norm;
    err_z = qerr_z * scale;
  }

  if (!z_enabled || fabs(err_z) < ANGLE_ERR_DEADBAND) err_z = 0.0f;

  last_err_z = err_z;

  bool attitudeQuiet = err_z == 0.0f;
  bool rateQuiet = wz == 0.0f;

  if (attitudeQuiet && rateQuiet) {
    int_z = 0.0f;
    last_tau_z = 0.0f;
    last_ramp_z = 0.0f;
    sendADCSCommand(0.0f);
    logADCSCommand();
    return ADCS_PERIOD_US;
  }

  updateYawController(err_z, wz);

  if (CAM::isTransmitting()) {
    last_ramp_z = 0.0f;
    sendADCSCommand(0.0f);
  } else {
    sendADCSCommand(last_ramp_z);
  }

  logADCSCommand();

  return ADCS_PERIOD_US;
}

// ==========================
// TELEMETRY TASKS
// ==========================

// ADCS_TELEMETRY (53 bytes):
//   setpoint quat (4xf) | current quat (4xf) | wz | int_z |
//   z angle error | z torque command | z ramp command | mode (uint8, 0=track 1=detumble)
uint32_t task_sendADCSTelem() {
  if (CAM::isTransmitting()) return 1000000;
  Packet p;
  p.id = ADCS_TELEMETRY;
  p.length = 0;
  RadioComms::packetAddFloat(&p, q_des_w);
  RadioComms::packetAddFloat(&p, q_des_x);
  RadioComms::packetAddFloat(&p, q_des_y);
  RadioComms::packetAddFloat(&p, q_des_z);
  RadioComms::packetAddFloat(&p, last_qw);
  RadioComms::packetAddFloat(&p, last_qx);
  RadioComms::packetAddFloat(&p, last_qy);
  RadioComms::packetAddFloat(&p, last_qz);
  RadioComms::packetAddFloat(&p, last_wz);
  RadioComms::packetAddFloat(&p, int_z);
  RadioComms::packetAddFloat(&p, last_err_z);
  RadioComms::packetAddFloat(&p, last_tau_z);
  RadioComms::packetAddFloat(&p, last_ramp_z);
  RadioComms::packetAddUint8(&p, detumbling ? 1 : 0);
  RadioComms::emitPacket(&p);
  return 1000000; // 1 Hz
}

// ADCS_PARAMS (13 bytes):
//   Z Kp/Ki/Kd (3xf) | z enabled (uint8)
uint32_t task_sendADCSParams() {
  if (CAM::isTransmitting()) return 5000000;
  Packet p;
  p.id = ADCS_PARAMS;
  p.length = 0;
  RadioComms::packetAddFloat(&p, KP_Z);
  RadioComms::packetAddFloat(&p, KI_Z);
  RadioComms::packetAddFloat(&p, KD_Z);
  RadioComms::packetAddUint8(&p, z_enabled ? 1 : 0);
  RadioComms::emitPacket(&p);
  return 5000000; // 0.2 Hz
}

}
