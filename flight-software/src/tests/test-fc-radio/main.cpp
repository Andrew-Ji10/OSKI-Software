#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "../../flight-controller/Common.h"

#define CS_PIN    10
#define RESET_PIN 9
#define SPI_MOSI  11
#define SPI_MISO  12
#define SPI_SCK   13
#define EN_PIN    14
#define LED1      3
#define LED2      46

static const byte LOCAL_ADDR = 0xAA;
static const byte BROADCAST  = 0xFF;

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
  delay(2000);
  Serial.begin(115200);
  while (!Serial);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, HIGH);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);

  pinMode(CS_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(10);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CS_PIN);
  LoRa.setPins(CS_PIN, RESET_PIN, -1);

  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }

  Serial.println("LoRa init succeeded.");
}

void loop() {
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

  if (recipient != LOCAL_ADDR && recipient != (int)BROADCAST) {
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

  if (packet.id == CMD_PING && packet.length >= 4) {
    uint32_t arg = getUint32(&packet, 0);
    Serial.printf("CMD_PING from 0x%02X  arg: %lu\n", sender, arg);

    Packet response;
    response.id = CMD_PING;
    response.length = 0;
    addUint32(&response, arg);
    addUint32(&response, 42);

    sendPacket(&response, sender);
    digitalWrite(LED1, !digitalRead(LED1));
    digitalWrite(LED2, !digitalRead(LED2));
  }
}
