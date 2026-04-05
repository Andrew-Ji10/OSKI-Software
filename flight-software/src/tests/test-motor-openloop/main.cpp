#include <Arduino.h>
#include <SimpleFOC.h>

#define IN1 33
#define IN2 15
#define IN3 32
#define SIMPLEFOC_EN 14
#define SIMPLEFOC_GND 27
#include <SimpleFOC.h>

// BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(7);
// BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(IN1, IN2, IN3, SIMPLEFOC_EN); // set your pins
uint32_t last_time = 0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12; //set your power supply voltage;
  // limit the maximal dc voltage the driver can set
  driver.voltage_limit = 6; //set your voltage limit; Usually half of power supply voltage is a good 

  pinMode(SIMPLEFOC_GND, OUTPUT); // set "GND" pin to LOW to be SimpleFOC's GND
  digitalWrite(SIMPLEFOC_GND, LOW); // set "GND" pin

  pinMode(LED_BUILTIN, OUTPUT); // set the LED pin as output
  digitalWrite(LED_BUILTIN, LOW); // turn off the LED

  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 3; //set your voltage limit in volts;   // [V]
 
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

  // set the target velocity [rad/s]
  motor.target = 6.28; // one rotation per second

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {

  // loop FOC algorithm, should be called as 
  // frequently as possible for best 
  // performance (e.g. 1kHz+)
  motor.loopFOC();

  // open loop velocity movement
  motor.move();

  // user communication
  command.run();

  // blink LED at 1Hz to signal that the program is running
  if (millis() - last_time > 500) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // toggle the LED
    last_time = millis();
  }
}