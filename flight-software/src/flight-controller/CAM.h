#ifndef CAM_H
#define CAM_H
#include <Arduino.h>

namespace CAM {
  void init();
  uint32_t task_processCamera();
  bool isTransmitting();
}
#endif
