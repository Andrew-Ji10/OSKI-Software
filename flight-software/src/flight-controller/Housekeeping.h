#ifndef HOUSEKEEPING_H
#define HOUSEKEEPING_H
#include <Arduino.h>

namespace Housekeeping {
  inline void init() {}
  inline uint32_t task_sendHSK() { return 1000 * 1000; }
}
#endif
