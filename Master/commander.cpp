#include <SPI.h>
#include <RF24.h>
#include <Thread.h>
#include <Arduino.h>

#include "_config.h"
#include "_types.h"

#include "robot.h"
#include "system.h"
#include "motors.h"
#include "commander.h"
#include "controller.h"

//
// NRF24L01 Config
//
// RH_NRF24 *radio;//(PIN_RADIO_CE, PIN_RADIO_CSN);
// RHReliableDatagram *radioManager;//(radio, 2);

uint8_t radioData[] = "Hi!";

uint8_t radioBuffer[64];
uint8_t radioBufferOut[64];

// Checks for communication and sets robot's state to IDDLE/ACTIVE
void threadIddleDetection_run();
Thread threadIddleDetection(threadIddleDetection_run, 1000);

// Serial communication (DEBUG)
void threadSerial_run();
Thread threadSerial(threadSerial_run, 50);

// NRF Communication
void threadNRF_run();
Thread threadNRF(threadNRF_run, 1000); // Starts listening after 1sec, then set itself to 5ms


// ====================================
//           INITIALIZATION
// ====================================

void Commander::init(){

  LOG("Commander::init\n");

  // Initialize Radio
  LOG("Radio::init\n");
  // radio = new RH_NRF24(PIN_RADIO1_CE, PIN_RADIO1_CSN);

  // radioManager = new RHReliableDatagram(*radio, 2);
  // radioManager->setRetries(1);
  // radioManager->setTimeout(3);
  // radioManager->setThisAddress(Robot::getRobotID());

  // if(!radio->init()){
  //   Robot::alarm = ALARM_INIT;
  //   LOG(" ! Radio failed to init\n");
  // }

  // if (!radio->setChannel(Robot::getRobotID()))
    // LOG(" ! setChannel failed\n");

  // if (!radio->setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    // LOG(" ! setRF failed\n");

  system.add(&threadIddleDetection);
  system.add(&threadSerial);
  system.add(&threadNRF);
}



// ====================================
//           MESSAGE PARSING
// ====================================

/*
  Handles the Message from buffer, set's the output buffer,
  and returns the length to send back.
*/
bool Commander::handleMessage(uint8_t message[], uint8_t len){
  bool activate = false;
  char cmd;

  // LOG("Got [");LOG(len);LOG("]: ");
  // for(int i = 0; i < len; i++){
  //   Serial.print((int)message[i]);
  //   Serial.print(" ");
  // }

  if(len <= 0)
    return 0;

  // Get first byte (as the command)
  cmd = message[0];

  if(cmd == 'g'){
    // Get Robot Status Packet:
    //  [0] = STATE (IDDLE/ACTIVE)
    //  [1] = ALARM
    //  [2] = Battery voltage (x10)

    activate = false;
  }

  if(cmd == 's'){
    // Stops the robot
    Robot::setState(IDDLE);

    activate = false;
  }

  if(cmd == 'a'){
    // if(len != 7){
      // LOG("Invalid command with pattern.");
      // return 0;
    // }

    // Set Robot target Theta/Position
    /*
      Incoming Pattern:
        [0] = 'a'

        [1] = thetaValL
        [2] = thetaValH

        [3] = yValL
        [4] = yValH

      Out Pattern:
        [0] = AA;
    */
    uint16_t rawTheta = message[1] | (message[2] << 8);
    float Theta = ((float)rawTheta - 3600) / 10.0;

    uint16_t rawY = message[3] | (message[4] << 8);
    float Y = ((float)rawY - 1000) / 10.0;

    // Update actual target
    Controller::setTarget(0, Y, Theta);

    // Raise flag to activate robot (Go out of IDDLE)
    activate = true;

    // LOG("SET TARGET STATE: \n");
    // LOG("y: "); LOG(Controller::targetY); LOG(" ["); LOG(Controller::targetYType); LOG("]\n");
    // LOG("d: "); LOG(Controller::targetTheta); LOG(" ["); LOG(Controller::targetThetaType); LOG("]\n");

  }

  if(cmd == 'p'){
    // Set Robot PID
    /*
      Incoming Pattern:
        [0] = 'p'

        [1] = kp * 10
        [2] = ki * 10
        [3] = kd * 10
        [3] = kiLimit * 10

      Out Pattern:
        [0] = AA;
    */

    int yType = message[4];
    Controller::setPIDConstants(
      message[1] / 10.0,
      message[2] / 10.0,
      message[3] / 10.0,
      message[4]
    );

    // LOG("SET TARGET STATE: \n");
    // LOG("y: "); LOG(Controller::targetY); LOG(" ["); LOG(Controller::targetYType); LOG("]\n");
    // LOG("d: "); LOG(Controller::targetTheta); LOG(" ["); LOG(Controller::targetThetaType); LOG("]\n");

    activate = false;
  }

  return activate;
}


// ====================================
//          THREAD CALLBACKS
// ====================================

void threadIddleDetection_run(){
  // Only checks if state of robot is not ACTIVE
  if(Robot::state != ACTIVE)
    return;

  if(millis() - Robot::lastTimeActive > RADIO_TIMEOUT_TO_IDDLE){
    Robot::setState(IDDLE);
    Robot::doBeep(Robot::getRobotID(), 50);
  }
}

void threadSerial_run(){
  if(!Serial.available())
    return;

  char got = Serial.read();

  LOG("got:"); LOG(got); LOG("\n");

  if(got == '1')
    Motors::setPower(100, 0);

  else if(got == '2')
    Motors::setPower(-100, 0);

  else if(got == '3')
    Motors::setPower(0, 100);

  else if(got == '4')
    Motors::setPower(0, -100);

  else if(got == '8'){
    Controller::setTarget(0, 0, -90);
  }else if(got == '0'){
    Controller::setTarget(0, 0, 0);
    Motors::stop();
  }else if(got == 'b'){
    LOG("bat: "); LOG(Robot::vbat); LOG("v\n");

  }else if(got == 'i'){
    LOG("ID: "); LOG(Robot::getRobotID()); LOG("\n");

  }else if(got == '@'){
    LOG(F("===========================\n"));
    LOG(F("= Serial Interactive Mode =\n"));
    LOG(F("= Send  ':q'  to exit     =\n"));
    LOG(F("===========================\n"));

    bool notExited = true;
    char lastChr = ' ';
    char chr;
    while(notExited){
      // Pipe incoming data from USB Serial to Slave Serial
      while(Serial.available()){
        chr = Serial.read();
        if(chr == 'q' && lastChr == ':')
          notExited = false;
        lastChr = chr;
        Serial1.write(chr);
      }

      // Pipe incoming data from Slave Serial to USB Serial
      while(Serial1.available())
        Serial.write(Serial1.read());
    }
  }else if(got == 'h'){
    // LOG("\n---- help ----\n");
    // LOG("0: Stop motors\n");
    // LOG("b: Get bat. voltage\n");
    // LOG("i: robot ID\n");
  }
}

void threadNRF_run(){
  long start = millis();

  // Set interval to 5ms
  threadNRF.setInterval(0);

  // if(!radio->available())
  //   return;

  uint8_t len = sizeof(radioBuffer);

  // if (!radio->recv(radioBuffer, &len)){
  //   LOG(" ! recv");
  //   return;
  // }

  // LOG("Got msg! ");LOG(len);LOG("\n");
  bool activate = Commander::handleMessage(radioBuffer, len);

  radioBufferOut[0] = Robot::state;
  radioBufferOut[1] = Robot::alarm;
  radioBufferOut[2] = Robot::vbat*10;

  // radio->send(radioBufferOut, 3);
  // radio->waitPacketSent();

  // Got message. Robot is now Active if message was an action message
  if(Robot::alarm == NONE){
    if(Robot::state == IDDLE && activate){
      Robot::setState(ACTIVE);
      Robot::doBeep(1, 100);
  }

    // Save timestamp of message
    Robot::lastTimeActive = millis();
    // Serial.print(" ");
    // Serial.print(lastMessageTimestamp - start);
    // Serial.print(" ms\n");
  }
}
