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
static const uint32_t HOST_BAUD = 115200;
static const size_t MAX_LORA_PAYLOAD = 251;
static const size_t SERIAL_HEX_BUF_LEN = (MAX_LORA_PAYLOAD + 4) * 2 + 1;
static const uint8_t FRAME_SYNC_0 = 0xA5;
static const uint8_t FRAME_SYNC_1 = 0x5A;

char serialBuf[SERIAL_HEX_BUF_LEN];
size_t serialLen = 0;
bool serialPacketReady = false;

void emitSerialPacket(uint8_t id, const uint8_t *data, uint8_t length, int16_t rssi, int8_t snrQuarterDb) {
  uint8_t frame[2 + 1 + 1 + 2 + 1 + MAX_LORA_PAYLOAD + 1];
  size_t idx = 0;
  uint8_t checksum = 0;

  frame[idx++] = FRAME_SYNC_0;
  frame[idx++] = FRAME_SYNC_1;

  frame[idx++] = id;
  checksum += id;

  frame[idx++] = length;
  checksum += length;

  frame[idx++] = static_cast<uint8_t>(rssi & 0xFF);
  checksum += frame[idx - 1];
  frame[idx++] = static_cast<uint8_t>((rssi >> 8) & 0xFF);
  checksum += frame[idx - 1];

  frame[idx++] = static_cast<uint8_t>(snrQuarterDb);
  checksum += frame[idx - 1];

  for (uint8_t i = 0; i < length; i++) {
    frame[idx++] = data[i];
    checksum += data[i];
  }

  frame[idx++] = checksum;
  Serial.write(frame, idx);
}

void trySendSerialPacketToLoRa() {
  if (!serialPacketReady) return;

  while (serialLen > 0 && isspace(static_cast<unsigned char>(serialBuf[serialLen - 1]))) {
    serialLen--;
  }

  if (serialLen < 2) {
    serialLen = 0;
    serialPacketReady = false;
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
  serialPacketReady = false;
}

void processSerialInput() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      serialPacketReady = (serialLen > 0);
      continue;
    }

    if (c == '\r') continue;

    if (serialPacketReady) {
      continue;
    }

    if (serialLen + 1 < SERIAL_HEX_BUF_LEN) {
      serialBuf[serialLen++] = c;
      serialBuf[serialLen] = '\0';
    } else {
      serialLen = 0;
      serialPacketReady = false;
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

  int16_t rssi = static_cast<int16_t>(LoRa.packetRssi());
  int8_t snrQuarterDb = static_cast<int8_t>(LoRa.packetSnr() * 4.0f);

  // Re-enter RX mode before any host forwarding so we do not miss the next packet.
  LoRa.receive();

  if (recipient != LOCAL_ADDR && recipient != 0xFF) return true;

  emitSerialPacket(id, data, n, rssi, snrQuarterDb);

  (void)sender;
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  return true;
}

void setup() {
  Serial.begin(HOST_BAUD);
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
  LoRa.enableCrc();
  LoRa.receive();
}

void loop() {
  processSerialInput();
  trySendSerialPacketToLoRa();
  while (processLoRaPacket()) {
    processSerialInput();
    trySendSerialPacketToLoRa();
  }
}
