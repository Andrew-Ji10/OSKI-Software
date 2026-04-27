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

static const int PING_INTERVAL_MS = 5000;

long lastPingTime = 0;
uint32_t pingArg = 0;

void sendPacket(Packet *packet, byte dest);
void onReceive(int packetSize);
void printPacket(Packet *packet, byte sender, int rssi, float snr);

static void addUint32(Packet *p, uint32_t v) {
  p->data[p->length]   = v & 0xFF;
  p->data[p->length+1] = v >> 8 & 0xFF;
  p->data[p->length+2] = v >> 16 & 0xFF;
  p->data[p->length+3] = v >> 24 & 0xFF;
  p->length += 4;
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
    Serial.println("# LoRa init failed. Check your connections.");
    while (true);
  }

  Serial.println("# Ground station ready.");
}

void loop() {
  if (millis() - lastPingTime > PING_INTERVAL_MS) {
    Packet ping;
    ping.id = CMD_PING;
    ping.length = 0;
    addUint32(&ping, pingArg);
    sendPacket(&ping, FC_ADDR);
    Serial.printf("# Sent CMD_PING arg=%lu\n", pingArg);
    pingArg++;
    lastPingTime = millis();
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

  printPacket(&packet, sender, LoRa.packetRssi(), LoRa.packetSnr());
}

// DATA <id> <len> <byte0> <byte1> ... RSSI=<n> SNR=<n>
void printPacket(Packet *packet, byte sender, int rssi, float snr) {
  Serial.printf("DATA %d %d", packet->id, packet->length);
  for (uint8_t i = 0; i < packet->length; i++) {
    Serial.printf(" %02X", packet->data[i]);
  }
  Serial.printf(" RSSI=%d SNR=%.1f\n", rssi, snr);
}
