#ifndef GPS_H
#define GPS_H
#include <Arduino.h>

namespace GPS {
  inline void init() {}
  inline uint32_t task_readGPS() { return 1000 * 1000; }
}
#endif
