#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO_SDA 8
#define BNO_SCL 7

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
  Serial.begin(921600);
  delay(1000);

  Wire.begin(BNO_SDA, BNO_SCL);
  Wire.setClock(400000);

  if (!bno.begin()) {
    Serial.println("BNO055 not found");
    while (1) {
      delay(1000);
    }
  }

  bno.setExtCrystalUse(true);
  delay(500);
}

void loop() {
  static uint32_t last_us = 0;
  const uint32_t period_us = 20000;  // 50 Hz output

  uint32_t now = micros();
  if (now - last_us < period_us) {
    return;
  }
  last_us = now;

  imu::Quaternion q = bno.getQuat();

  Serial.print(q.w(), 6);
  Serial.print(",");
  Serial.print(q.x(), 6);
  Serial.print(",");
  Serial.print(q.y(), 6);
  Serial.print(",");
  Serial.println(q.z(), 6);
}