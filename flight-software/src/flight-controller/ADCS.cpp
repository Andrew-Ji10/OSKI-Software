#include "ADCS.h"
#include "CAM.h"

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

// Quaternion PID gains (mutable for in-flight tuning via CMD_ADCS_SET_PID).
static float KP_X = 1.0e-3f;
static float KP_Y = 1.0e-3f;
static float KP_Z = 1.0e-3f;

static float KI_X = 0.0f;
static float KI_Y = 0.0f;
static float KI_Z = 0.0f;

static float KD_X = 2.0e-3f;
static float KD_Y = 2.0e-3f;
static float KD_Z = 2.0e-3f;

// Integrator windup clamp [rad·s] (quaternion error vector units)
static constexpr float I_MAX = 0.1f;

// Per-axis integrator accumulators
static float int_x = 0.0f;
static float int_y = 0.0f;
static float int_z = 0.0f;

// Last measured attitude + gyro (updated each ADCS tick, read by telem task)
static float last_qw = 1.0f, last_qx = 0.0f, last_qy = 0.0f, last_qz = 0.0f;
static float last_wx = 0.0f, last_wy = 0.0f, last_wz = 0.0f;

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
  RadioComms::registerCallback(CMD_ADCS_SET_PID, setPIDGains);

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