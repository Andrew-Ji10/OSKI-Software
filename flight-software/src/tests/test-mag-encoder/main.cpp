#include<Arduino.h>
#include <SPI.h>


#define CS_PIN 4   //  change to your actual CS pin

uint16_t readAngleRaw() {
  uint16_t command = 0xFFFF;  // NOP to trigger read
  uint16_t response = 0;

  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);

  response = SPI.transfer16(command);

  digitalWrite(CS_PIN, HIGH);

  // mask to 14 bits (AS5048A resolution)
  return response & 0x3FFF;
}

void setup() {
  Serial.begin(115200);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin(SCK, 21, 19);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); // AS5048A = MODE1

  Serial.println("AS5048A SPI test starting...");
}

void loop() {
  uint16_t raw = readAngleRaw();

  float angle_deg = (raw * 360.0) / 16384.0;

  Serial.print("Raw: ");
  Serial.print(raw);
  Serial.print("\tAngle (deg): ");
  Serial.println(angle_deg);

  delay(10);
}