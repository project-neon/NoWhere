#include <Thread.h>
#include <Arduino.h>
#include <RH_NRF24.h>
// #include <RHReliableDatagram.h>

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
RH_NRF24 *radio;//(PIN_RADIO_CE, PIN_RADIO_CSN);
// RHReliableDatagram *radioManager;//(radio, 2);

uint8_t radioData[] = "Hi!";

uint8_t radioBuffer[RH_NRF24_MAX_MESSAGE_LEN];
uint8_t radioBufferOut[RH_NRF24_MAX_MESSAGE_LEN];

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
  radio = new RH_NRF24(PIN_RADIO_CE, PIN_RADIO_CSN);

  // radioManager = new RHReliableDatagram(*radio, 2);
  // radioManager->setRetries(1);
  // radioManager->setTimeout(3);
  // radioManager->setThisAddress(Robot::getRobotID());

  if(!radio->init()){
    Robot::alarm = ALARM_INIT;
    LOG(" ! Radio failed to init\n");
  }

  if (!radio->setChannel(Robot::getRobotID()))
    LOG(" ! setChannel failed\n");

  if (!radio->setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    LOG(" ! setRF failed\n");

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

        [1] = thetaType (0, 1, 2...)
        [2] = thetaValL
        [3] = thetaValH

        [4] = yType
        [5] = yValL
        [6] = yValH

      Out Pattern:
        [0] = AA;
    */
    int thetaType = message[1];
    uint16_t thetaRaw = message[2] | (message[3] << 8);

    int yType = message[4];
    uint16_t yRaw = message[5] | (message[6] << 8);

    if(thetaType > 2 || yType > 2){
      LOG("Invalid type");
      return 0;
    }

    // Controller::setTargetY(static_cast<TargetType>(2), ((float)yRaw - 1800) / 10.0);
    Controller::setTargetY(RATE_OF_CHANGE, ((float)yRaw - 1000) / 10.0);
    // Controller::setTargetTheta(static_cast<TargetType>(1), ((float)thetaRaw - 1800) / 10.0);
    Controller::setTargetTheta(RATE_OF_CHANGE, ((float)thetaRaw - 3600) / 10.0);

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
    Motors::setPower(50, 0);

  else if(got == '2')
    Motors::setPower(-50, 0);

  else if(got == '3')
    Motors::setPower(0, 50);

  else if(got == '4')
    Motors::setPower(0, -50);

  else if(got == '0')
    Motors::stop();

  else if(got == 'b'){
    LOG("bat: "); LOG(Robot::vbat); LOG("v\n");

  }else if(got == 'i'){
    LOG("ID: "); LOG(Robot::getRobotID()); LOG("\n");

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

  if(!radio->available())
    return;

  uint8_t len = sizeof(radioBuffer);

  if (!radio->recv(radioBuffer, &len)){
    LOG(" ! recv");
    return;
  }

  // LOG("Got msg! ");LOG(len);LOG("\n");
  bool activate = Commander::handleMessage(radioBuffer, len);

  radioBufferOut[0] = Robot::state;
  radioBufferOut[1] = Robot::alarm;
  radioBufferOut[2] = Robot::vbat*10;

  radio->send(radioBufferOut, 3);
  radio->waitPacketSent();

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
