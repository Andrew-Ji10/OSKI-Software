#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#define LORA_SCK   7
#define LORA_MISO  21
#define LORA_MOSI  19
#define LORA_CS     5
#define LORA_RST   4
#define LORA_IRQ   8

volatile bool gotPacket = false;

void onReceive(int packetSize) {
  if (packetSize > 0) {
    gotPacket = true;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa init failed");
    while (true) {}
  }

  LoRa.onReceive(onReceive);
  LoRa.receive();

  Serial.println("LoRa RX ready");
}

void loop() {
  if (gotPacket) {
    gotPacket = false;
    Serial.print("Received: ");
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
    Serial.println();
    LoRa.receive();
  }

  delay(1000);
  Serial.print("Working \n");
}