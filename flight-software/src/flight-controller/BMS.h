#ifndef BMS_H
#define BMS_H
#include <Arduino.h>

namespace BMS {
  inline void init() {}
  inline uint32_t task_readBMS() { return 1000 * 1000; }
}
#endif
