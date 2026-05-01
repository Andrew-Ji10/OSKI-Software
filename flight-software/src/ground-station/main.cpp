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
static const size_t MAX_LORA_PAYLOAD = 251;
static const size_t SERIAL_HEX_BUF_LEN = (MAX_LORA_PAYLOAD + 4) * 2 + 1;
static const size_t SERIAL_LINE_BUF_LEN = 32 + MAX_LORA_PAYLOAD * 3 + 32;

char serialBuf[SERIAL_HEX_BUF_LEN];
size_t serialLen = 0;

void sendSerialPacketToLoRa() {
  while (serialLen > 0 && isspace(static_cast<unsigned char>(serialBuf[serialLen - 1]))) {
    serialLen--;
  }

  if (serialLen < 2) {
    serialLen = 0;
    return;
  }

  if (LoRa.beginPacket() == 0) {
    return;
  }

  for (size_t i = 0; i + 1 < serialLen; i += 2) {
    char hex[3] = { serialBuf[i], serialBuf[i + 1], '\0' };
    LoRa.write(static_cast<uint8_t>(strtoul(hex, nullptr, 16)));
  }
  LoRa.endPacket();
  LoRa.receive();
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  serialLen = 0;
}

void processSerialInput() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      sendSerialPacketToLoRa();
      continue;
    }

    if (c == '\r') continue;

    if (serialLen + 1 < SERIAL_HEX_BUF_LEN) {
      serialBuf[serialLen++] = c;
      serialBuf[serialLen] = '\0';
    } else {
      serialLen = 0;
    }
  }
}

bool processLoRaPacket() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return false;

  if (packetSize < 4) {
    while (LoRa.available()) LoRa.read();
    LoRa.receive();
    return true;
  }

  int recipient = LoRa.read();
  byte sender   = static_cast<byte>(LoRa.read());
  uint8_t id    = static_cast<uint8_t>(LoRa.read());
  uint8_t length = static_cast<uint8_t>(LoRa.read());

  uint8_t data[MAX_LORA_PAYLOAD];
  uint8_t n = 0;
  while (LoRa.available() && n < length) {
    data[n++] = static_cast<uint8_t>(LoRa.read());
  }
  while (LoRa.available()) LoRa.read();

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  // Re-enter RX mode before any serial formatting so we do not miss the next packet.
  LoRa.receive();

  if (recipient != LOCAL_ADDR && recipient != 0xFF) return true;

  char line[SERIAL_LINE_BUF_LEN];
  int used = snprintf(line, sizeof(line), "DATA %u %u", id, n);
  for (uint8_t i = 0; i < n && used > 0 && used < static_cast<int>(sizeof(line)); i++) {
    used += snprintf(line + used, sizeof(line) - used, " %02X", data[i]);
  }

  if (used > 0 && used < static_cast<int>(sizeof(line))) {
    used += snprintf(line + used, sizeof(line) - used, " RSSI=%d SNR=%.1f\n", rssi, snr);
  }

  if (used > 0) {
    size_t toWrite = static_cast<size_t>(used);
    if (toWrite > sizeof(line)) toWrite = sizeof(line);
    Serial.write(reinterpret_cast<const uint8_t *>(line), toWrite);
  }

  (void)sender;
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  return true;
}

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
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(500E3);
  LoRa.setCodingRate4(5);
  LoRa.receive();
  Serial.println("# Ground station ready.");
}

void loop() {
  processSerialInput();
  while (processLoRaPacket()) {
    processSerialInput();
  }
}
