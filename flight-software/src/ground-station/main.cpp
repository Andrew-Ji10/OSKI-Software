#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "../flight-controller/Common.h"

#define CS_PIN    5
#define RESET_PIN 4
#define SPI_MOSI  19
#define SPI_MISO  21
#define SPI_SCK   7
#define G0        8

static const byte LOCAL_ADDR = 0xBB;
static const byte FC_ADDR    = 0xAA;

String serialBuf = "";

void setup() {
  Serial.begin(115200);
  while (!Serial);
  pinMode(LED_BUILTIN, OUTPUT);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CS_PIN);
  LoRa.setPins(CS_PIN, RESET_PIN, G0);
  if (!LoRa.begin(915E6)) {
    Serial.println("# LoRa init failed.");
    while (true);
  }
  Serial.println("# Ground station ready.");
}

void loop() {
  // Serial -> LoRa: relay hex-encoded bytes from host
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      serialBuf.trim();
      if (serialBuf.length() >= 2) {
        LoRa.beginPacket();
        for (int i = 0; i + 1 < (int)serialBuf.length(); i += 2) {
          char hex[3] = { serialBuf[i], serialBuf[i + 1], '\0' };
          LoRa.write((uint8_t)strtoul(hex, nullptr, 16));
        }
        LoRa.endPacket();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
      serialBuf = "";
    } else {
      serialBuf += c;
    }
  }

  // LoRa -> Serial: forward packet content as DATA line
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  int     recipient = LoRa.read();
  byte    sender    = LoRa.read();
  uint8_t id        = LoRa.read();
  uint8_t length    = LoRa.read();

  if (recipient != LOCAL_ADDR && recipient != 0xFF) {
    while (LoRa.available()) LoRa.read();
    return;
  }

  uint8_t data[251];
  uint8_t n = 0;
  while (LoRa.available() && n < length) data[n++] = LoRa.read();

  Serial.printf("DATA %d %d", id, n);
  for (uint8_t i = 0; i < n; i++) Serial.printf(" %02X", data[i]);
  Serial.printf(" RSSI=%d SNR=%.1f\n", LoRa.packetRssi(), LoRa.packetSnr());
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
