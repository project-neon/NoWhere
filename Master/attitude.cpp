#include <Thread.h>
#include <Arduino.h>
#include <ThreadController.h>

#include "_config.h"
#include "_types.h"
#include "OdometryPacket.h"

#include "robot.h"
#include "system.h"
#include "attitude.h"


#define ODOMETRY_PACKET_SIZE    5

// Checks for incoming packets from Odometry
void threadOdometry_run();
Thread threadOdometry(threadOdometry_run, 1);

void Attitude::init(){
  LOG("Attitude::init\n");

  Serial1.begin(115200);

  system.add(&threadOdometry);
}

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
      Robot::dt,
      Robot::inclinated,
      Robot::onFloor);

    if(success){
      Serial.print(Robot::onFloor);
      Serial.print("\t");
      Serial.print(Robot::inclinated);
      Serial.print("\t");
      Serial.print(Robot::dx);
      Serial.print("\t");
      Serial.print(Robot::dy);
      Serial.print("\t");
      Serial.print(Robot::dt);
      Serial.println();
    }
  }
}
