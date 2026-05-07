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

static constexpr float JW_X = 3.11e-6f;
static constexpr float JW_Y = 3.11e-6f;
static constexpr float JW_Z = 3.11e-6f;

static constexpr float TAU_MAX = 3.11e-4f;
static constexpr float RAMP_MAX = 100.0f;

static float KP_X = 2.0e-4f;
static float KP_Y = 2.0e-4f;
static float KP_Z = 2.0e-4f;

static float KD_X = 2.0e-4f;
static float KD_Y = 2.0e-4f;
static float KD_Z = 2.0e-4f;

static float KI_X = 0.0f;
static float KI_Y = 0.0f;
static float KI_Z = 0.0f;

static constexpr float GYRO_DEADBAND = 0.20f;
static constexpr float QERR_DEADBAND = 0.03f;

// Integrator windup clamp [rad·s] (quaternion error vector units)
static constexpr float I_MAX = 0.1f;

// Per-axis integrator accumulators
static float int_x = 0.0f;
static float int_y = 0.0f;
static float int_z = 0.0f;

// Last measured attitude + gyro (updated each ADCS tick, read by telem task)
static float last_qw = 1.0f, last_qx = 0.0f, last_qy = 0.0f, last_qz = 0.0f;
static float last_wx = 0.0f, last_wy = 0.0f, last_wz = 0.0f;

static constexpr uint32_t ADCS_PERIOD_US = 20000;

// ==========================
// DESIRED ATTITUDE
// ==========================

static float q_des_w = 1.0f;
static float q_des_x = 0.0f;
static float q_des_y = 0.0f;
static float q_des_z = 0.0f;

// Y disabled by default for now
static bool x_enabled = true;
static bool y_enabled = false;
static bool z_enabled = true;

// ==========================
// HELPERS
// ==========================

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
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

  int_x = 0.0f; int_y = 0.0f; int_z = 0.0f; // reset integrators on setpoint change
  Packet response;
  response.id = CMD_ADCS_SETPOINT;
  response.length = 0;
  RadioComms::emitPacket(&response);
}

void enableADCS(Packet packet) {
  x_enabled = packet.data[0] != 0;
  y_enabled = packet.data[1] != 0;
  z_enabled = packet.data[2] != 0;

  Packet response;
  response.id = CMD_ADCS_ENABLE;
  response.length = 0;

  RadioComms::packetAddUint8(&response, x_enabled ? 1 : 0);
  RadioComms::packetAddUint8(&response, y_enabled ? 1 : 0);
  RadioComms::packetAddUint8(&response, z_enabled ? 1 : 0);

  RadioComms::emitPacket(&response);
}

// ==========================
// WHEEL COMMANDS
// ==========================

static void sendADCSCommand(float ramp_x, float ramp_y, float ramp_z) {
  if (x_enabled) {
    WHEEL_SERIAL.print("XR");
    WHEEL_SERIAL.println(ramp_x, 3);
  }

  if (y_enabled) {
    WHEEL_SERIAL.print("YR");
    WHEEL_SERIAL.println(ramp_y, 3);
  }

  if (z_enabled) {
    WHEEL_SERIAL.print("ZR");
    WHEEL_SERIAL.println(ramp_z, 3);
  }
}

static void zeroWheels() {
  WHEEL_SERIAL.println("XR0.000");
  WHEEL_SERIAL.println("YR0.000");
  WHEEL_SERIAL.println("ZR0.000");

  WHEEL_SERIAL.println("XV0");
  WHEEL_SERIAL.println("YV0");
  WHEEL_SERIAL.println("ZV0");
}

// CMD_ADCS_SET_PID payload: axis (uint8, 0=X 1=Y 2=Z) | Kp (float) | Ki (float) | Kd (float)
void setPIDGains(Packet packet) {
  uint8_t axis = RadioComms::packetGetUint8(&packet, 0);
  float kp     = RadioComms::packetGetFloat(&packet, 1);
  float ki     = RadioComms::packetGetFloat(&packet, 5);
  float kd     = RadioComms::packetGetFloat(&packet, 9);

  switch (axis) {
    case 0: KP_X = kp; KI_X = ki; KD_X = kd; int_x = 0.0f; break;
    case 1: KP_Y = kp; KI_Y = ki; KD_Y = kd; int_y = 0.0f; break;
    case 2: KP_Z = kp; KI_Z = ki; KD_Z = kd; int_z = 0.0f; break;
    default: break;
  }

  Serial.printf("PID axis %u: Kp=%.6f Ki=%.6f Kd=%.6f\n", axis, kp, ki, kd);

  Packet response;
  response.id = CMD_ADCS_SET_PID;
  response.length = 0;
  RadioComms::packetAddUint8(&response, axis);
  RadioComms::packetAddFloat(&response, kp);
  RadioComms::packetAddFloat(&response, ki);
  RadioComms::packetAddFloat(&response, kd);
  RadioComms::emitPacket(&response);
}

// CMD_ADCS_WHEEL_VEL payload: axis (uint8, 0=X 1=Y 2=Z) | velocity (float)
void setWheelVelocity(Packet packet) {
  uint8_t axis = RadioComms::packetGetUint8(&packet, 0);
  float vel    = RadioComms::packetGetFloat(&packet, 1);

  switch (axis) {
    case 0:
      x_enabled = false;
      WHEEL_SERIAL.print("XV");
      WHEEL_SERIAL.println(vel, 3);
      break;
    case 1:
      y_enabled = false;
      WHEEL_SERIAL.print("YV");
      WHEEL_SERIAL.println(vel, 3);
      break;
    case 2:
      z_enabled = false;
      WHEEL_SERIAL.print("ZV");
      WHEEL_SERIAL.println(vel, 3);
      break;
    default: return;
  }

  Packet response;
  response.id = CMD_ADCS_WHEEL_VEL;
  response.length = 0;
  RadioComms::packetAddUint8(&response, axis);
  RadioComms::packetAddFloat(&response, vel);
  RadioComms::emitPacket(&response);
}

void zeroWheelsCmd(Packet packet) {
  x_enabled = false;
  y_enabled = false;
  z_enabled = false;

  int_x = 0.0f; int_y = 0.0f; int_z = 0.0f;
  zeroWheels();

  Packet response;
  response.id = CMD_ADCS_ZERO;
  response.length = 0;
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

  float wx = gyro.x();
  float wy = gyro.y();
  float wz = gyro.z();

  if (fabs(wx) < GYRO_DEADBAND) wx = 0.0f;
  if (fabs(wy) < GYRO_DEADBAND) wy = 0.0f;
  if (fabs(wz) < GYRO_DEADBAND) wz = 0.0f;

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

  if (fabs(qerr_x) < QERR_DEADBAND) qerr_x = 0.0f;
  if (fabs(qerr_y) < QERR_DEADBAND) qerr_y = 0.0f;
  if (fabs(qerr_z) < QERR_DEADBAND) qerr_z = 0.0f;

  bool attitudeQuiet = (qerr_x == 0.0f && qerr_y == 0.0f && qerr_z == 0.0f);
  bool rateQuiet = (wx == 0.0f && wy == 0.0f && wz == 0.0f);

  if (attitudeQuiet && rateQuiet) {
    sendADCSCommand(0.0f, 0.0f, 0.0f);
    return ADCS_PERIOD_US;
  }


  // Integrate quaternion error vector (dt = ADCS_PERIOD_US in seconds)
  static constexpr float DT = ADCS_PERIOD_US * 1.0e-6f;
  int_x = clampf(int_x + qerr_x * DT, -I_MAX, I_MAX);
  int_y = clampf(int_y + qerr_y * DT, -I_MAX, I_MAX);
  int_z = clampf(int_z + qerr_z * DT, -I_MAX, I_MAX);

  // PID control:
  // tau_body = -Kp * q_error_vector - Ki * integral - Kd * omega
  float tau_x = -KP_X * qerr_x - KI_X * int_x - KD_X * wx;
  float tau_y = -KP_Y * qerr_y - KI_Y * int_y - KD_Y * wy;
  float tau_z = -KP_Z * qerr_z - KI_Z * int_z - KD_Z * wz;

  tau_x = clampf(tau_x, -TAU_MAX, TAU_MAX);
  tau_y = clampf(tau_y, -TAU_MAX, TAU_MAX);
  tau_z = clampf(tau_z, -TAU_MAX, TAU_MAX);

  float ramp_x = (-tau_x) / JW_X;
  float ramp_y = (-tau_y) / JW_Y;
  float ramp_z = (-tau_z) / JW_Z;

  ramp_x = clampf(ramp_x, -RAMP_MAX, RAMP_MAX);
  ramp_y = clampf(ramp_y, -RAMP_MAX, RAMP_MAX);
  ramp_z = clampf(ramp_z, -RAMP_MAX, RAMP_MAX);

  if (CAM::isTransmitting()) {
    sendADCSCommand(0.0f, 0.0f, 0.0f);
  } else {
    sendADCSCommand(ramp_x, ramp_y, ramp_z);
  }

  last_qw = qw; last_qx = qx; last_qy = qy; last_qz = qz;
  last_wx = wx; last_wy = wy; last_wz = wz;

  return ADCS_PERIOD_US;
}

// ==========================
// TELEMETRY TASKS
// ==========================

// ADCS_TELEMETRY (56 bytes):
//   setpoint quat (4×f) | current quat (4×f) | gyro (3×f) | integrators (3×f)
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
  RadioComms::packetAddFloat(&p, last_wx);
  RadioComms::packetAddFloat(&p, last_wy);
  RadioComms::packetAddFloat(&p, last_wz);
  RadioComms::packetAddFloat(&p, int_x);
  RadioComms::packetAddFloat(&p, int_y);
  RadioComms::packetAddFloat(&p, int_z);
  RadioComms::emitPacket(&p);
  return 1000000; // 1 Hz
}

// ADCS_PARAMS (39 bytes):
//   Kp/Ki/Kd per axis (9×f) | x/y/z enabled (3×uint8)
uint32_t task_sendADCSParams() {
  if (CAM::isTransmitting()) return 5000000;
  Packet p;
  p.id = ADCS_PARAMS;
  p.length = 0;
  RadioComms::packetAddFloat(&p, KP_X);
  RadioComms::packetAddFloat(&p, KI_X);
  RadioComms::packetAddFloat(&p, KD_X);
  RadioComms::packetAddFloat(&p, KP_Y);
  RadioComms::packetAddFloat(&p, KI_Y);
  RadioComms::packetAddFloat(&p, KD_Y);
  RadioComms::packetAddFloat(&p, KP_Z);
  RadioComms::packetAddFloat(&p, KI_Z);
  RadioComms::packetAddFloat(&p, KD_Z);
  RadioComms::packetAddUint8(&p, x_enabled ? 1 : 0);
  RadioComms::packetAddUint8(&p, y_enabled ? 1 : 0);
  RadioComms::packetAddUint8(&p, z_enabled ? 1 : 0);
  RadioComms::emitPacket(&p);
  return 5000000; // 0.2 Hz
}

}