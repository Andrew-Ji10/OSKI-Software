#include "ADCS.h"

namespace ADCS {

// ==========================
// BNO055 CONFIG
// ==========================

static constexpr int BNO_SDA = 8;
static constexpr int BNO_SCL = 7;
static constexpr uint8_t BNO_ADDR = 0x28;

static Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDR, &Wire);

#define WHEEL_SERIAL Serial // Replace later

// ==========================
// CONTROL CONSTANTS
// ==========================

// Reaction wheel inertia [kg m^2]
// Replace with your actual wheel inertia.
static constexpr float JW_X = 1.28e-5f;
static constexpr float JW_Y = 1.28e-5f;
static constexpr float JW_Z = 1.28e-5f;

// Body torque saturation [N m]
// Start VERY small for safety.
static constexpr float TAU_MAX = 2.0e-4f;

// Wheel ramp-rate saturation [rad/s^2]
static constexpr float RAMP_MAX = 20.0f;

// Quaternion PD gains.
// Start small. Tune slowly.
static constexpr float KP_X = 1.0e-3f;
static constexpr float KP_Y = 1.0e-3f;
static constexpr float KP_Z = 1.0e-3f;

static constexpr float KD_X = 2.0e-3f;
static constexpr float KD_Y = 2.0e-3f;
static constexpr float KD_Z = 2.0e-3f;

// ADCS task period
static constexpr uint32_t ADCS_PERIOD_US = 20000; // 50 Hz

// ==========================
// DESIRED ATTITUDE
// ==========================

// For first test: desired attitude is identity quaternion.
// This means "hold whatever the BNO frame calls zero attitude."
static float q_des_w = 1.0f;
static float q_des_x = 0.0f;
static float q_des_y = 0.0f;
static float q_des_z = 0.0f;

static bool x_enabled = true;
static bool y_enabled = true;
static bool z_enabled = true;

// ==========================
// HELPERS
// ==========================..................................................................................................................

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

// Quaternion conjugate
static void quatConj(float w, float x, float y, float z,
                     float &ow, float &ox, float &oy, float &oz) {
  ow = w;
  ox = -x;
  oy = -y;
  oz = -z;
}

// Quaternion multiply: q = a ⊗ b
static void quatMultiply(float aw, float ax, float ay, float az,
                         float bw, float bx, float by, float bz,
                         float &ow, float &ox, float &oy, float &oz) {
  ow = aw*bw - ax*bx - ay*by - az*bz;
  ox = aw*bx + ax*bw + ay*bz - az*by;
  oy = aw*by - ax*bz + ay*bw + az*bx;
  oz = aw*bz + ax*by - ay*bx + az*bw;
}

void setDesiredAttitude(Packet packet) {
  q_des_w = RadioComms::packetGetFloat(&packet, 0);
  q_des_x = RadioComms::packetGetFloat(&packet, 4);
  q_des_y = RadioComms::packetGetFloat(&packet, 8);
  q_des_z = RadioComms::packetGetFloat(&packet, 12);
  //ack
  Packet response;
  response.id = CMD_ADCS_SETPOINT;
  response.length = 0;
  RadioComms::emitPacket(&response);
}

void enableADCS(Packet packet) {
    x_enabled = packet.data[0] != 0;
    y_enabled = packet.data[1] != 0;
    z_enabled = packet.data[2] != 0;
    //ack
    Packet response;
    response.id = CMD_ADCS_ENABLE;
    response.length = 0;
    RadioComms::packetAddUint8(&response, x_enabled ? 1 : 0);
    RadioComms::packetAddUint8(&response, y_enabled ? 1 : 0);
    RadioComms::packetAddUint8(&response, z_enabled ? 1 : 0);
    RadioComms::emitPacket(&response);
}

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
  sendADCSCommand(0.0f, 0.0f, 0.0f);
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
// INIT
// ==========================

void init() {
  Wire.begin(BNO_SDA, BNO_SCL);
  Wire.setClock(100000); // More stable than 400k for BNO055 bring-up

  if (!bno.begin()) {
    Serial.println("ADCS ERR: BNO055 not found");
    return;
  }

  delay(100);
  bno.setExtCrystalUse(true);
  delay(100);

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

  // Body rates from BNO055 gyro [rad/s]
  float wx = gyro.x();
  float wy = gyro.y();
  float wz = gyro.z();

  // q_err = q_des^{-1} ⊗ q_current
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

  // Shortest-path quaternion error.
  // If scalar part is negative, flip sign.
  if (qerr_w < 0.0f) {
    qerr_w = -qerr_w;
    qerr_x = -qerr_x;
    qerr_y = -qerr_y;
    qerr_z = -qerr_z;
  }

  // PD control:
  // tau_body = -Kp * q_error_vector - Kd * omega
  float tau_x = -KP_X * qerr_x - KD_X * wx;
  float tau_y = -KP_Y * qerr_y - KD_Y * wy;
  float tau_z = -KP_Z * qerr_z - KD_Z * wz;

  tau_x = clampf(tau_x, -TAU_MAX, TAU_MAX);
  tau_y = clampf(tau_y, -TAU_MAX, TAU_MAX);
  tau_z = clampf(tau_z, -TAU_MAX, TAU_MAX);

  // Wheel torque is opposite body torque.
  float tau_wheel_x = -tau_x;
  float tau_wheel_y = -tau_y;
  float tau_wheel_z = -tau_z;

  // Convert wheel torque to wheel angular acceleration [rad/s^2]
  float ramp_x = tau_wheel_x / JW_X;
  float ramp_y = tau_wheel_y / JW_Y;
  float ramp_z = tau_wheel_z / JW_Z;

  ramp_x = clampf(ramp_x, -RAMP_MAX, RAMP_MAX);
  ramp_y = clampf(ramp_y, -RAMP_MAX, RAMP_MAX);
  ramp_z = clampf(ramp_z, -RAMP_MAX, RAMP_MAX);

  sendADCSCommand(ramp_x, ramp_y, ramp_z);

  return ADCS_PERIOD_US;
}

}