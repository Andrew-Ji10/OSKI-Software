#ifndef BMS_H
#define BMS_H
#include <Arduino.h>

namespace BMS {
  void init();
  uint32_t task_sendBMSTelem();
}
#endif
