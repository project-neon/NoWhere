#include <Thread.h>
#include <Arduino.h>
#include <ThreadController.h>

#include "_config.h"
#include "_types.h"
#include "OdometryPacket.h"

#include "robot.h"
#include "system.h"
#include "attitude.h"


#define ODOMETRY_PACKET_SIZE    9

// Checks for incoming packets from Odometry
void threadOdometry_run();
Thread threadOdometry(threadOdometry_run, 1);

void Attitude::init(){
  LOG("Attitude::init\n");

  Serial1.begin(115200);

  system.add(&threadOdometry);
}

bool Attitude::newData = false;

uint8_t buffer[ODOMETRY_PACKET_SIZE];
void threadOdometry_run(){

  // Check if any byte available
  while(Serial1.available()){
    unsigned long start = micros();
    // Move bytes together
    memmove(buffer + 0, buffer + 1, ODOMETRY_PACKET_SIZE - 1);

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

    unsigned long end = micros();

    // Serial.println(buffer[0], BIN);
    if(success){
      Attitude::newData = true;
      // Serial.print(Robot::onFloor);
      // Serial.print("\t");
      // Serial.print(Robot::inclinated);
      // Serial.print("\t");
      // Serial.print(Robot::dx);
      // Serial.print("\t");
      // Serial.print(Robot::dy);
      // Serial.print("\t");
      // Serial.print(Robot::theta);
      // Serial.println();
    }
  }
}
