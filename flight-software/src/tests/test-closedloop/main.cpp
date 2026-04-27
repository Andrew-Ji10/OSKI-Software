#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <math.h>

// ================= SHARED SPI PINS =================
#define SPI_SCK   14
#define SPI_MISO  22
#define SPI_MOSI  20

// ================= WHEEL X PINS =================
#define X_IN1 19
#define X_IN2 4
#define X_IN3 5
#define X_EN  26   // shared EN line for all three drivers
#define X_CS  32

// ================= WHEEL Y PINS =================
#define Y_IN1 7
#define Y_IN2 8
#define Y_IN3 21
#define Y_CS  15

// ================= WHEEL Z PINS =================
#define Z_IN1 27
#define Z_IN2 13
#define Z_IN3 12
#define Z_CS  33

// ================= MOTOR PARAMETERS =================
#define POLE_PAIRS       7
#define PHASE_RESISTANCE 7.1
#define KV_RATING        100
#define Q_AXIS_IND       0.00145
#define D_AXIS_IND       0.00145
#define WHEEL_INERTIA    1.28e-5   // kg*m^2, 304SS disk 40mm dia 1/4in thick

// ================= SAFETY LIMITS =================
#define MAX_WHEEL_SPEED  300.0
#define MAX_WHEEL_ACCEL  500.0
#define MAX_WHEEL_TORQUE 0.0030

// ================= OBJECTS =================
BLDCMotor motorX = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, KV_RATING, Q_AXIS_IND, D_AXIS_IND);
BLDCMotor motorY = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, KV_RATING, Q_AXIS_IND, D_AXIS_IND);
BLDCMotor motorZ = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, KV_RATING, Q_AXIS_IND, D_AXIS_IND);

// Only driverX owns EN=26. Y and Z omit the pin so they never fight the shared line.
BLDCDriver3PWM driverX = BLDCDriver3PWM(X_IN1, X_IN2, X_IN3, X_EN);
BLDCDriver3PWM driverY = BLDCDriver3PWM(Y_IN1, Y_IN2, Y_IN3);
BLDCDriver3PWM driverZ = BLDCDriver3PWM(Z_IN1, Z_IN2, Z_IN3);

MagneticSensorSPI sensorX = MagneticSensorSPI(X_CS, 14, 0x3FFF, 1000000);
MagneticSensorSPI sensorY = MagneticSensorSPI(Y_CS, 14, 0x3FFF, 1000000);
MagneticSensorSPI sensorZ = MagneticSensorSPI(Z_CS, 14, 0x3FFF, 1000000);

SPIClass SPI_AS5048(VSPI);
Commander command = Commander(Serial);

// ================= STATE =================
float speedCmdX = 0.0, speedCmdY = 0.0, speedCmdZ = 0.0;
float torqueCmdX = 0.0, torqueCmdY = 0.0, torqueCmdZ = 0.0;
uint32_t lastUpdateX = 0, lastUpdateY = 0, lastUpdateZ = 0;
uint32_t lastPrint = 0;
bool print_enabled = false;

// ================= HELPERS =================
float clampFloat(float x, float lo, float hi) {
  return x > hi ? hi : x < lo ? lo : x;
}

void printWheelStatus(const char* name, BLDCMotor& motor, MagneticSensorSPI& sensor,
                      float speedCmd, float torqueCmd) {
  Serial.print(name);
  Serial.print(" | τ_cmd: ");     Serial.print(torqueCmd, 6);
  Serial.print(" N*m  v_cmd: ");  Serial.print(speedCmd, 3);
  Serial.print(" rad/s  v_act: "); Serial.print(sensor.getVelocity(), 3);
  Serial.print(" rad/s  θ: ");    Serial.print(sensor.getAngle(), 3);
  Serial.print(" rad  Vlim: ");   Serial.println(motor.voltage_limit, 2);
}

void printAllStatus() {
  printWheelStatus("X", motorX, sensorX, speedCmdX, torqueCmdX);
  printWheelStatus("Y", motorY, sensorY, speedCmdY, torqueCmdY);
  printWheelStatus("Z", motorZ, sensorZ, speedCmdZ, torqueCmdZ);
}

void updateWheel(BLDCMotor& motor, float& speedCmd, float torqueCmd, uint32_t& lastUpdateUs) {
  uint32_t now_us = micros();
  if (lastUpdateUs == 0) { lastUpdateUs = now_us; return; }
  float dt = (now_us - lastUpdateUs) * 1e-6f;
  lastUpdateUs = now_us;
  if (dt <= 0 || dt > 0.1f) return;
  float accelCmd = clampFloat(torqueCmd / WHEEL_INERTIA, -MAX_WHEEL_ACCEL, MAX_WHEEL_ACCEL);
  speedCmd = clampFloat(speedCmd + accelCmd * dt, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
  motor.target = speedCmd;
}

void setupWheel(BLDCMotor& motor, BLDCDriver3PWM& driver, MagneticSensorSPI& sensor,
                int cs, const char* name) {
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);
  sensor.init(&SPI_AS5048);

  driver.voltage_power_supply = 12.0;
  driver.voltage_limit = 12.0;
  if (!driver.init()) {
    Serial.print(name); Serial.println(" driver init FAILED");
    while (1);
  }

  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller        = MotionControlType::velocity;
  motor.voltage_limit        = 3.0;
  motor.voltage_sensor_align = 1.0;  // reduces inrush during FOC alignment
  motor.velocity_limit       = MAX_WHEEL_SPEED;
  motor.PID_velocity.P  = 0.08;
  motor.PID_velocity.I  = 0.5;
  motor.PID_velocity.D  = 0.0;
  motor.LPF_velocity.Tf = 0.03;
  motor.useMonitoring(Serial);
  motor.init();

  Serial.print("Aligning "); Serial.println(name);
  motor.initFOC();
  motor.target = 0.0;
}

// ================= MOTOR DISPATCH =================
// Commander calls doX("V100"), doX("T0.001"), doX("B"), doX("S"), doX("") etc.
void dispatchMotor(BLDCMotor& motor, MagneticSensorSPI& sensor,
                   float& speedCmd, float& torqueCmd, uint32_t& lastUpdate,
                   const char* name, char* cmd) {
  if (!cmd || cmd[0] == '\0') {
    printWheelStatus(name, motor, sensor, speedCmd, torqueCmd);
    return;
  }
  char  sub = toupper((unsigned char)cmd[0]);
  char* arg = cmd + 1;
  switch (sub) {
    case 'T':
      torqueCmd = clampFloat(atof(arg), -MAX_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
      lastUpdate = micros();
      Serial.print(name); Serial.print(" torque: "); Serial.print(torqueCmd, 6); Serial.println(" N*m");
      break;
    case 'V':
      torqueCmd = 0.0;
      speedCmd  = clampFloat(atof(arg), -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
      motor.target = speedCmd;
      lastUpdate = micros();
      Serial.print(name); Serial.print(" speed: "); Serial.print(speedCmd, 3); Serial.println(" rad/s");
      break;
    case 'B':
      torqueCmd    = 0.0;
      speedCmd     = 0.0;
      motor.target = 0.0;
      lastUpdate   = micros();
      Serial.print(name); Serial.println(" braking");
      break;
    case 'S':
      printWheelStatus(name, motor, sensor, speedCmd, torqueCmd);
      break;
    default:
      Serial.print("Unknown sub-cmd '"); Serial.print(cmd[0]);
      Serial.println("'  valid: T V B S");
  }
}

void doX(char* cmd) { dispatchMotor(motorX, sensorX, speedCmdX, torqueCmdX, lastUpdateX, "X", cmd); }
void doY(char* cmd) { dispatchMotor(motorY, sensorY, speedCmdY, torqueCmdY, lastUpdateY, "Y", cmd); }
void doZ(char* cmd) { dispatchMotor(motorZ, sensorZ, speedCmdZ, torqueCmdZ, lastUpdateZ, "Z", cmd); }

// ================= GLOBAL COMMANDS =================
void doEnableAll(char* cmd) {
  // Snapshot current speeds for bumpless re-enable
  sensorX.update(); speedCmdX = sensorX.getVelocity(); torqueCmdX = 0.0; motorX.target = speedCmdX;
  sensorY.update(); speedCmdY = sensorY.getVelocity(); torqueCmdY = 0.0; motorY.target = speedCmdY;
  sensorZ.update(); speedCmdZ = sensorZ.getVelocity(); torqueCmdZ = 0.0; motorZ.target = speedCmdZ;
  lastUpdateX = lastUpdateY = lastUpdateZ = micros();
  motorX.enable();  // drives EN=26 HIGH, physically enables all three drivers
  motorY.enable();  // sets enabled flag only (no EN pin on driverY)
  motorZ.enable();  // sets enabled flag only (no EN pin on driverZ)
  Serial.println("All enabled.");
}

void doDisableAll(char* cmd) {
  motorX.disable();  // drives EN=26 LOW, physically disables all three drivers
  motorY.disable();
  motorZ.disable();
  Serial.println("All disabled.");
}

void doPrintToggle(char* cmd) {
  print_enabled = !print_enabled;
  Serial.print("Periodic print "); Serial.println(print_enabled ? "ON" : "OFF");
}

void doAllStatus(char* cmd) { printAllStatus(); }

void doHelp(char* cmd) {
  Serial.println(F("\n=== COMMAND REFERENCE ==="));
  Serial.println(F("Per-motor  (axis = X, Y, or Z):"));
  Serial.println(F("  <axis>T<val>   torque (N*m)      e.g.  XT0.001   ZT-0.002"));
  Serial.println(F("  <axis>V<val>   speed  (rad/s)    e.g.  YV100     XV-50"));
  Serial.println(F("  <axis>B        brake  (hold 0)   e.g.  XB"));
  Serial.println(F("  <axis>S        status             e.g.  ZS"));
  Serial.println(F("Global:"));
  Serial.println(F("  E    enable  all motors"));
  Serial.println(F("  D    disable all motors"));
  Serial.println(F("  S    status  all motors"));
  Serial.println(F("  P    toggle  1 Hz status print"));
  Serial.println(F("  H    this help message"));
  Serial.println(F("========================="));
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println(F("\n=== THREE-WHEEL REACTION WHEEL CONTROL ==="));

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  SPI_AS5048.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  // Init order places X in MCPWM group 0, Z in group 1, Y shares group 0 with X.
  // This avoids the X+Z group-sharing issue while keeping X+Y (known good) together.
  setupWheel(motorX, driverX, sensorX, X_CS, "X");
  setupWheel(motorZ, driverZ, sensorZ, Z_CS, "Z");
  setupWheel(motorY, driverY, sensorY, Y_CS, "Y");

  lastUpdateX = micros();
  lastUpdateY = micros();
  lastUpdateZ = micros();

  command.add('X', doX, "X motor");
  command.add('Y', doY, "Y motor");
  command.add('Z', doZ, "Z motor");
  command.add('E', doEnableAll,    "Enable all");
  command.add('D', doDisableAll,   "Disable all");
  command.add('S', doAllStatus,    "All status");
  command.add('P', doPrintToggle,  "Toggle print");
  command.add('H', doHelp,         "Help");

  doHelp(nullptr);
}

// ================= LOOP =================
void loop() {
  updateWheel(motorX, speedCmdX, torqueCmdX, lastUpdateX);
  updateWheel(motorY, speedCmdY, torqueCmdY, lastUpdateY);
  updateWheel(motorZ, speedCmdZ, torqueCmdZ, lastUpdateZ);

  motorX.loopFOC();
  motorY.loopFOC();
  motorZ.loopFOC();

  motorX.move();
  motorY.move();
  motorZ.move();

  command.run();

  if (print_enabled && millis() - lastPrint > 1000) {
    lastPrint = millis();
    printAllStatus();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
