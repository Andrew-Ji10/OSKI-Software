#ifndef IMU_H
#define IMU_H
#include <Arduino.h>

namespace IMU {
  inline void init() {}
  inline uint32_t task_readIMU() { return 10 * 1000; }
}
#endif
