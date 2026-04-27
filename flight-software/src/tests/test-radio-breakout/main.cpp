#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "../../flight-controller/Common.h"

#define CS_PIN    5
#define RESET_PIN 4
#define SPI_MOSI  19
#define SPI_MISO  21
#define SPI_SCK   7
#define G0        8

static const byte LOCAL_ADDR = 0xBB;
static const byte FC_ADDR    = 0xAA;

long lastSendTime = 0;
int interval = 2000;
uint32_t pingArg = 0;

void sendPacket(Packet *packet, byte dest);
void onReceive(int packetSize);

static void addUint32(Packet *p, uint32_t v) {
  p->data[p->length]   = v & 0xFF;
  p->data[p->length+1] = v >> 8 & 0xFF;
  p->data[p->length+2] = v >> 16 & 0xFF;
  p->data[p->length+3] = v >> 24 & 0xFF;
  p->length += 4;
}

static uint32_t getUint32(Packet *p, uint8_t i) {
  uint32_t v = p->data[i+3];
  v = (v << 8) | p->data[i+2];
  v = (v << 8) | p->data[i+1];
  v = (v << 8) | p->data[i];
  return v;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(CS_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(10);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CS_PIN);
  LoRa.setPins(CS_PIN, RESET_PIN, G0);

  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }

  Serial.println("LoRa init succeeded.");
}

void loop() {
  if (millis() - lastSendTime > interval) {
    Packet ping;
    ping.id = CMD_PING;
    ping.length = 0;
    addUint32(&ping, pingArg);

    sendPacket(&ping, FC_ADDR);
    Serial.printf("Sent CMD_PING  arg: %lu\n", pingArg);
    pingArg++;
    lastSendTime = millis();
    interval = random(2000) + 1000;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  onReceive(LoRa.parsePacket());
}

void sendPacket(Packet *packet, byte dest) {
  if (packet->length > 251) return;
  LoRa.beginPacket();
  LoRa.write(dest);
  LoRa.write(LOCAL_ADDR);
  LoRa.write(packet->id);
  LoRa.write(packet->length);
  for (uint8_t i = 0; i < packet->length; i++) {
    LoRa.write(packet->data[i]);
  }
  LoRa.endPacket();
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  int recipient  = LoRa.read();
  byte sender    = LoRa.read();
  uint8_t id     = LoRa.read();
  uint8_t length = LoRa.read();

  if (recipient != LOCAL_ADDR && recipient != 0xFF) {
    while (LoRa.available()) LoRa.read();
    return;
  }

  Packet packet;
  packet.id = id;
  packet.length = 0;
  for (uint8_t i = 0; i < length && LoRa.available(); i++) {
    packet.data[i] = LoRa.read();
    packet.length++;
  }

  if (packet.id == CMD_PING && packet.length >= 8) {
    uint32_t echoedArg = getUint32(&packet, 0);
    uint32_t val       = getUint32(&packet, 4);
    Serial.printf("Ping response — echoed: %lu  val: %lu  RSSI: %d  SNR: %.1f\n",
      echoedArg, val, LoRa.packetRssi(), LoRa.packetSnr());
  } else {
    Serial.printf("Received from: 0x%02X  id: %d  length: %d  RSSI: %d  SNR: %.1f\n",
      sender, packet.id, packet.length, LoRa.packetRssi(), LoRa.packetSnr());
  }
}
