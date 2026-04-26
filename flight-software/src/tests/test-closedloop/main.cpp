#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>

// ========= MOTOR DRIVER PINS =========
#define IN1 33
#define IN2 15
#define IN3 32
#define SIMPLEFOC_EN 14
#define SIMPLEFOC_GND 27

// ========= AS5048A SPI PINS =========
#define CS_PIN   4
#define SPI_SCK  SCK     // use board's actual SCK pin
#define SPI_MISO 21
#define SPI_MOSI 19

// ========= MOTOR / DRIVER =========
// PM2205 likely 6 pole pairs. If it behaves badly, try 7.
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(IN1, IN2, IN3, SIMPLEFOC_EN);

// ========= SENSOR =========
MagneticSensorSPI sensor = MagneticSensorSPI(CS_PIN, 14, 0x3FFF, 1000000);
SPIClass SPI_AS5048(VSPI);

// ========= COMMANDER =========
Commander command = Commander(Serial);

void doTarget(char* cmd) {
  command.scalar(&motor.target, cmd);
}

void doLimit(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
}

uint32_t last_print = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n=== CLOSED LOOP VELOCITY CONTROL ===");

  // Fake GND pin from your setup
  pinMode(SIMPLEFOC_GND, OUTPUT);
  digitalWrite(SIMPLEFOC_GND, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // ========= SENSOR INIT =========
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI_AS5048.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CS_PIN);
  sensor.init(&SPI_AS5048);

  // ========= DRIVER INIT =========
  driver.voltage_power_supply = 12.0;
  driver.voltage_limit = 3.0;

  if (!driver.init()) {
    Serial.println("Driver init failed.");
    while (1);
  }

  // ========= LINK MOTOR =========
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  // ========= CONTROL MODE =========
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity;

  // ========= SAFETY LIMITS =========
  motor.voltage_limit = 1.5;      // start low
  motor.velocity_limit = 10.0;    // rad/s

  // ========= STARTER PID VALUES =========
  motor.PID_velocity.P = 0.15;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0.0;
  motor.LPF_velocity.Tf = 0.02;

  // ========= MONITORING =========
  motor.useMonitoring(Serial);

  // ========= INIT =========
  motor.init();

  Serial.println("Running FOC alignment...");
  motor.initFOC();

  // Start at zero. Use Serial command T2 to spin.
  motor.target = 0.0;

  command.add('T', doTarget, "target velocity rad/s");
  command.add('L', doLimit, "voltage limit");

  Serial.println("Motor ready.");
  Serial.println("Commands:");
  Serial.println("T2    -> spin at 2 rad/s");
  Serial.println("T-2   -> spin reverse at -2 rad/s");
  Serial.println("T0    -> stop");
  Serial.println("L1.0  -> set voltage limit to 1V");
  Serial.println("L2.0  -> set voltage limit to 2V");
}

void loop() {
  motor.loopFOC();
  motor.move();

  command.run();

  if (millis() - last_print > 250) {
    last_print = millis();

    Serial.print("target: ");
    Serial.print(motor.target);
    Serial.print("\tangle: ");
    Serial.print(sensor.getAngle());
    Serial.print("\tvelocity: ");
    Serial.print(sensor.getVelocity());
    Serial.print("\tvoltage_limit: ");
    Serial.println(motor.voltage_limit);

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}