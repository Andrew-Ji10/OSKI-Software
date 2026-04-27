#include <Arduino.h>
#include <HardwareSerial.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <TinyGPS++.h>

#define GPS_RESETN 17
#define GPS_RX     16   // GPS RX (ESP32 TX)
#define GPS_TX     15   // GPS TX (ESP32 RX)
#define LED1       3
#define LED2       46

HardwareSerial GPSSerial(1);
SFE_UBLOX_GNSS ublox;
TinyGPSPlus gps;

void setup() {
  delay(2000);
  Serial.begin(115200);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, HIGH);

  pinMode(GPS_RESETN, OUTPUT);
  digitalWrite(GPS_RESETN, LOW);
  delay(10);
  digitalWrite(GPS_RESETN, HIGH);

  GPSSerial.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);  // esp_rx=GPS_TX, esp_tx=GPS_RX
  delay(1500);

  Serial.println("Connecting to GPS...");
  if (!ublox.begin(GPSSerial)) {
    Serial.println("GPS not found. Check wiring.");
    while (true);
  }

  // Restore NMEA output — previous session saved UBX-only to flash
  ublox.setUART1Output(COM_TYPE_NMEA);
  Serial.println("GPS init succeeded. Waiting for fix...");
  // TinyGPS++ takes over from here
}

void loop() {
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 1000) return;
  lastPrint = millis();

  uint8_t sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

  if (!gps.location.isValid()) {
    Serial.printf("Searching... Sats: %d\n", sats);
    return;
  }

  Serial.printf("Fix  Sats: %d  Lat: %.7f  Lon: %.7f  Alt: %.2fm\n",
    sats, gps.location.lat(), gps.location.lng(), gps.altitude.meters());
  digitalWrite(LED1, !digitalRead(LED1));
  digitalWrite(LED2, !digitalRead(LED2));
}
