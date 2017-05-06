// System Includes

// External Libraries
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <Thread.h>
#include <ThreadController.h>


// Custom Util
#include "_types.h"
#include "_config.h"

// Modules
#include "pid.h"
#include "robot.h"
#include "motors.h"
#include "system.h"
#include "attitude.h"
#include "commander.h"
#include "controller.h"

void setup(){

  // Initialize Core system stuff
  System::init();

  // Initialize Robot state and I/O Pins
  Robot::init();

  // Initialize Attitude/Orientation module (initializes MPU6050 internally)
  Attitude::init();

  // Initialize Communication
  Commander::init();

  // Initalize Motors (object)
  Motors::init();

  // Initialize Motor controllers (with PID's)
  Controller::init();

}

// long start, end;
void loop(){
  controller.run();
}
