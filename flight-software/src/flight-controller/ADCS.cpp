#include "ADCS.h"
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

static constexpr float KP_X = 2.0e-4f;
static constexpr float KP_Y = 2.0e-4f;
static constexpr float KP_Z = 2.0e-4f;

static constexpr float KD_X = 2.0e-4f;
static constexpr float KD_Y = 2.0e-4f;
static constexpr float KD_Z = 2.0e-4f;

static constexpr float GYRO_DEADBAND = 0.20f;
static constexpr float QERR_DEADBAND = 0.03f;

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

void zeroWheelsCmd(Packet packet) {
  x_enabled = false;
  y_enabled = false;
  z_enabled = false;

  zeroWheels();

  Packet response;
  response.id = CMD_ADCS_ZERO;
  response.length = 0;
  RadioComms::emitPacket(&response);
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
  Serial.println("Target: gyro>=3, accel>=2, mag>=2. Full ideal is 3/3/3/3.");

  const uint32_t timeout_ms = 30000;
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

    if (gyro >= 3 && accel >= 1 && mag >= 2) {
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

  waitForBNOCalibration();

  delay(300);
  setCurrentAttitudeAsTarget();

  RadioComms::registerCallback(CMD_ADCS_SETPOINT, setDesiredAttitude);
  RadioComms::registerCallback(CMD_ADCS_ENABLE, enableADCS);
  RadioComms::registerCallback(CMD_ADCS_ZERO, zeroWheelsCmd);

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

  float tau_x = -KP_X * qerr_x - KD_X * wx;
  float tau_y = -KP_Y * qerr_y - KD_Y * wy;
  float tau_z = -KP_Z * qerr_z - KD_Z * wz;

  tau_x = clampf(tau_x, -TAU_MAX, TAU_MAX);
  tau_y = clampf(tau_y, -TAU_MAX, TAU_MAX);
  tau_z = clampf(tau_z, -TAU_MAX, TAU_MAX);

  float ramp_x = (-tau_x) / JW_X;
  float ramp_y = (-tau_y) / JW_Y;
  float ramp_z = (-tau_z) / JW_Z;

  ramp_x = clampf(ramp_x, -RAMP_MAX, RAMP_MAX);
  ramp_y = clampf(ramp_y, -RAMP_MAX, RAMP_MAX);
  ramp_z = clampf(ramp_z, -RAMP_MAX, RAMP_MAX);

  sendADCSCommand(ramp_x, ramp_y, ramp_z);

  return ADCS_PERIOD_US;
}

}