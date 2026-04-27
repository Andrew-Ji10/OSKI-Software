#include "RadioComms.h"
#include <SPI.h>

namespace RadioComms
{
    static const int CS_PIN    = 10;
    static const int RESET_PIN = 9;
    static const int SPI_MOSI  = 11;
    static const int SPI_MISO  = 12;
    static const int SPI_SCK   = 13;
    static const int EN_PIN    = 14;

    static const byte LOCAL_ADDR = 0xAA;  // FC address
    static const byte BROADCAST  = 0xFF;

    std::map<uint8_t, commFunction> callbackMap;

    bool init() {
        pinMode(EN_PIN, OUTPUT);
        digitalWrite(EN_PIN, HIGH);

        pinMode(CS_PIN, OUTPUT);
        pinMode(RESET_PIN, OUTPUT);
        digitalWrite(RESET_PIN, LOW);
        delay(10);

        SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CS_PIN);
        LoRa.setPins(CS_PIN, RESET_PIN, -1);

        return LoRa.begin(915E6);
    }

    void registerCallback(uint8_t id, commFunction function) {
        callbackMap.insert(std::pair<int, commFunction>(id, function));
    }

    void evokeCallbackFunction(Packet *packet)
    {
        auto it = callbackMap.find(packet->id);
        if (it != callbackMap.end()) {
            it->second(*packet);
        }
    }

    void processWaitingPackets() {
        int packetSize = LoRa.parsePacket();
        if (packetSize == 0) return;

        int recipient   = LoRa.read();
        byte sender     = LoRa.read();
        uint8_t id      = LoRa.read();
        uint8_t length  = LoRa.read();

        if (recipient != LOCAL_ADDR && recipient != (int)BROADCAST) return;

        Packet packet;
        packet.id = id;
        packet.length = 0;
        for (uint8_t i = 0; i < length && LoRa.available(); i++) {
            packet.data[i] = LoRa.read();
            packet.length++;
        }

        evokeCallbackFunction(&packet);
    }

    void emitPacket(Packet *packet) {
        if (packet->length > 251) return;
        LoRa.beginPacket();
        LoRa.write(BROADCAST);
        LoRa.write(LOCAL_ADDR);
        LoRa.write(packet->id);
        LoRa.write(packet->length);
        for (uint8_t i = 0; i < packet->length; i++) {
            LoRa.write(packet->data[i]);
        }
        LoRa.endPacket();
    }

    void packetAddFloat(Packet *packet, float value)
    {
        uint32_t rawData = *(uint32_t *)&value;
        packet->data[packet->length] = rawData & 0xFF;
        packet->data[packet->length + 1] = rawData >> 8 & 0xFF;
        packet->data[packet->length + 2] = rawData >> 16 & 0xFF;
        packet->data[packet->length + 3] = rawData >> 24 & 0xFF;
        packet->length += 4;
    }

    void packetAddUint32(Packet *packet, uint32_t value)
    {
        packet->data[packet->length] = value & 0xFF;
        packet->data[packet->length + 1] = value >> 8 & 0xFF;
        packet->data[packet->length + 2] = value >> 16 & 0xFF;
        packet->data[packet->length + 3] = value >> 24 & 0xFF;
        packet->length += 4;
    }

    void packetAddUint16(Packet *packet, uint16_t value)
    {
        packet->data[packet->length] = value & 0xFF;
        packet->data[packet->length + 1] = value >> 8 & 0xFF;
        packet->length += 2;
    }

    void packetAddUint8(Packet *packet, uint8_t value)
    {
        packet->data[packet->length] = value;
        packet->length++;
    }

    float packetGetFloat(Packet *packet, uint8_t index)
    {
        uint32_t rawData = packet->data[index + 3];
        rawData <<= 8;
        rawData += packet->data[index + 2];
        rawData <<= 8;
        rawData += packet->data[index + 1];
        rawData <<= 8;
        rawData += packet->data[index];
        return *(float *)&rawData;
    }

    uint32_t packetGetUint32(Packet *packet, uint8_t index)
    {
        uint32_t rawData = packet->data[index + 3];
        rawData <<= 8;
        rawData += packet->data[index + 2];
        rawData <<= 8;
        rawData += packet->data[index + 1];
        rawData <<= 8;
        rawData += packet->data[index];
        return rawData;
    }

    uint32_t packetGetUint8(Packet *packet, uint8_t index)
    {
        return packet->data[index];
    }

}