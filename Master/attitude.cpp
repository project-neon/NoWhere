#include <Thread.h>
#include <Arduino.h>
#include <ThreadController.h>

#include "_config.h"
#include "_types.h"
#include "OdometryPacket.h"

#include "robot.h"
#include "system.h"
#include "attitude.h"

// Checks for incoming packets from Odometry each 1ms
void threadOdometry_run();
Thread threadOdometry(threadOdometry_run, 1);

// ====================================
//           INITIALIZATION
// ====================================

void Attitude::init(){
  
  // Initializes the Thread and the second Serial Interface
  
  LOG("Attitude::init\n");

  Serial1.begin(115200);

  controller.add(&threadOdometry);
}

// ====================================
//          THREAD CALLBACKS
// ====================================

// Flag for new data 
bool Attitude::newData = false;

// Buffer to hold Serial Data
uint8_t buffer[ODOMETRY_PACKET_SIZE];

// Odometry Function
void threadOdometry_run(){
  // Check if any byte available
  while(Serial1.available()){

    // Move bytes together
    memmove(buffer, buffer + 1, ODOMETRY_PACKET_SIZE - 1);

    // Put at the end
    buffer[ODOMETRY_PACKET_SIZE - 1] = Serial1.read();

    // Check if is a valid packet
    bool success = readOdometryPacket(
      buffer,
      Robot::dx,
      Robot::dy,
      Robot::theta,
      Robot::inclinated,
      Robot::onFloor);

    // If packet is OK then make the new data flag go true
    if(success){
      Attitude::newData = true;
    }
  }
}
