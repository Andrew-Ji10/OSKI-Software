#ifndef ADCS_H
#define ADCS_H
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Common.h"
#include "RadioComms.h"


namespace ADCS {
  void init();
  uint32_t task_runADCS();
}
#endif
