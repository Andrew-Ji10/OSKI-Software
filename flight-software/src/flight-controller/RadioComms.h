#ifndef RADIO_COMMS_H
#define RADIO_COMMS_H

#include <Arduino.h>
#include "Common.h"
#include <LoRa.h>

#include <map>
#include <vector>

namespace RadioComms
{
  bool init();

  typedef void (*commFunction)(Packet);

  /**
   * @brief Registers methods to be called when Comms receives a packet with a specific ID.
   *
   * @param id The ID of the packet associated with a specific command.
   * @param function a pointer to a method that takes in a Packet struct.
   */
  void registerCallback(uint8_t id, commFunction function);

  void processWaitingPackets();

  void packetAddFloat(Packet *packet, float value);
  void packetAddUint32(Packet *packet, uint32_t value);
  void packetAddUint16(Packet *packet, uint16_t value);
  void packetAddUint8(Packet *packet, uint8_t value);

  /**
   * @brief Interprets the packet data as a float.
   *
   * @param packet
   * @param index The index of the byte array at which the float starts (0, 4, 8).
   * @return float
   */
  float packetGetFloat(Packet *packet, uint8_t index);
  uint32_t packetGetUint32(Packet *packet, uint8_t index);
  uint32_t packetGetUint8(Packet *packet, uint8_t index);

  /**
   * @brief Sends packet data over Radio.
   *
   * @param packet The packet in which the data is stored.
   */
  void emitPacket(Packet *packet);

}
#endif