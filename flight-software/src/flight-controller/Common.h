#ifndef COMMON_H
#define COMMON_H
#include <Arduino.h>

// rough draft of packet IDs
#define CMD_PING 0
#define CMD_TAKE_PHOTO 1
#define CMD_ADCS_SETPOINT 2

#define HSK_TELEMETRY 100
#define BMS_TELEMETRY 101
// TODO MORE IDs

struct Task {
  uint32_t (*taskCall)();
  uint32_t nexttime;
  bool enabled;
};

// variable length packet structure
struct Packet {
  uint8_t id;
  uint8_t length;
  // timestamp?
  // checksum? nah will just append to end
  uint8_t data[251];  // 255-byte LoRa max minus 4-byte header
};

#endif