#ifndef COMMON_H
#define COMMON_H
#include <Arduino.h>

// rough draft of packet IDs
#define CMD_PING 0
#define CMD_TAKE_PHOTO 1
#define CMD_ADCS_SETPOINT 2
#define CMD_ADCS_ENABLE 6
#define CMD_ADCS_ZERO   7
#define CMD_ADCS_SET_PID 8
#define CMD_DEPLOY 3
#define CMD_SET_CAMERA_RES 4
#define CMD_CTRL_5V 5
#define CMD_RESET   9


#define HSK_TELEMETRY  100
#define BMS_TELEMETRY  101
#define ADCS_TELEMETRY 102  // setpoint quat + current quat + gyro + integrators (56 bytes)
#define ADCS_PARAMS    103  // PID gains per axis + enables (39 bytes)

#define CAM_IMAGE_META 200  // transfer_id (uint8) + total_size (uint32) + num_chunks (uint16)
#define CAM_IMAGE_DATA 201  // transfer_id (uint8) + seq (uint16) + total (uint16) + chunk bytes
#define CAM_IMAGE_DONE 202  // transfer_id (uint8), sender finished this image
#define CAM_NACK       203  // transfer_id (uint8) + missing seq list (uint16 each)
#define CAM_IMAGE_ACK  204  // transfer_id (uint8), image fully received

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
