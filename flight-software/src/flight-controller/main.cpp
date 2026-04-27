#include <Arduino.h>
#include "Common.h"
#include "BMS.h"
#include "IMU.h"
#include "GPS.h"
#include "RadioComms.h"
#include "CAM.h"
#include "Housekeeping.h"
#include "ADCS.h"

// Common files are included here, as well as specific files for this board

/**
 * @brief This code implements a task system that allows for the execution of multiple tasks at different intervals.
 * 
 * The task system is defined by the `Task` struct, which contains a function pointer to the task function, 
 * the next execution time of the task, and a flag indicating whether the task is enabled or not.
 * 
 * The `taskTable` array holds all the tasks that need to be executed. Each task is defined as a function that 
 * returns a `uint32_t` value representing the delay until the next execution of the task.
 * 
 * In the `setup()` function, the task system is initialized by setting up the necessary components and registering 
 * the task functions. The main loop in the `setup()` function iterates over all the tasks in the `taskTable` array 
 * and executes the tasks if their next execution time has been reached.
 * 
 * The `loop()` function is left empty as it is not used in this code.
 */

#define LED1 3
#define LED2 46
void initLEDs() {
    // initialize LEDs here
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, HIGH);
}

uint32_t task_blinkLEDs() {
    digitalWrite(LED1, !digitalRead(LED1));
    digitalWrite(LED2, !digitalRead(LED2));
    return 1000 * 1000; // this task will run every second
}

uint32_t task_helloWorld() {
  Serial.println("Hello World!");
  return 1000 * 1000; // this task will run every second
}

Task taskTable[] = {
  //{task_helloWorld, 0, true},
  {task_blinkLEDs, 0, true},
  // Example tasks, please flesh out with real tasks and remove these placeholders.
  {IMU::task_readIMU, 0, true},
  {GPS::task_readGPS, 0, true},
  {BMS::task_sendBMSTelem, 0, true},
  {Housekeeping::task_sendHSK, 0, true},
  {ADCS::task_runADCS, 0, true}
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

/*
Code from other files generally touches main in two places: an init function in setup, where they can register callbacks as to run functions
when recieving specific packets, and whatever code they need to run regularly that is put in the task table in main. 
But this means the main.cpp file is pretty light, and the other files are doing most of the work. 
(Means code is modular and related code is close together)
*/

void ping(Packet packet) {
  uint32_t arg = RadioComms::packetGetUint32(&packet, 0);
  Serial.printf("Received ping  arg: %lu\n", arg);
  Packet response;
  response.id = CMD_PING;
  response.length = 0;
  RadioComms::packetAddUint32(&response, arg);
  RadioComms::packetAddUint32(&response, 42);
  RadioComms::emitPacket(&response);
}

void setup() {
  // setup stuff here
  Serial.begin(115200);
  initLEDs();
  RadioComms::init();
  IMU::init();
  GPS::init();
  BMS::init();
  CAM::init();
  Housekeeping::init();

  // This way, RadioComms doesn't need to #include all other files to deal with commands
  RadioComms::registerCallback(CMD_PING, ping);


  while(1) {
    // main loop here to avoid arduino overhead
    for(uint32_t i = 0; i < TASK_COUNT; i++) { // for each task, execute if next time >= current time
      uint32_t ticks = micros(); // current time in microseconds
      if (taskTable[i].nexttime - ticks > UINT32_MAX / 2 && taskTable[i].enabled) {
        uint32_t delayoftask = taskTable[i].taskCall();
        if (delayoftask == 0) {
          taskTable[i].enabled = false;
        }
        else {
          taskTable[i].nexttime = ticks + delayoftask;
        }
      }
    }
    RadioComms::processWaitingPackets();
  }
}

void loop() {

} // unused
