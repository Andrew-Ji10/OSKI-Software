#ifndef ADCS_H
#define ADCS_H
#include <Arduino.h>

namespace ADCS {
  inline void init() {}
  inline uint32_t task_runADCS() { return 10 * 1000; }
}
#endif
