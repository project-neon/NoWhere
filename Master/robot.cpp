#include <Arduino.h>
#include <EEPROM.h>
#include <Thread.h>

#include "_config.h"
#include "_types.h"

#include "robot.h"
#include "motors.h"
#include "system.h"

void threadBeeper_run();
Thread threadBeeper(threadBeeper_run, 0);


// ====================================
//           INITIALIZERS
// ====================================
void Robot::init(){

  LOG("Robot::init\n");

  // Initialize Output Pins
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);

  pinMode(PIN_BUZZER, OUTPUT);

  // Initialize Input pins
  pinMode(PIN_BTN, INPUT);
  pinMode(PIN_VBAT, INPUT);

  // Setup State/Alarms
  Robot::setState(IDDLE);
  Robot::setAlarm(NONE);

  // Setup Beeper Thread
  controller.add(&threadBeeper);
  Robot::setBeep(BEEP_NONE);
};


// ====================================
//   SPECIFIC CONFIGURATIONS (EEPROM)
// ====================================

char _id = 0x00;
char Robot::getRobotID(){
  if(_id == 0x00){
    // Load from Eeprom if not loaded yet
    _id = EEPROM.read(EEPROM_ROBOT_ID);
  }
  return _id;
}

void Robot::setRobotID(char id){
  // Clear cache (Force re-reading)
  _id = 0x00;
  EEPROM.write(EEPROM_ROBOT_ID, id);
}


// ====================================
//         PRIMITIVE STATES
// ====================================
bool Robot::debug = false;
RobotState Robot::state = IDDLE;
RobotAlarm Robot::alarm = NONE;
unsigned long Robot::lastTimeActive = 0;

// Sets the robot's state
void Robot::setState(RobotState _state){
  state = _state;

  if(_state == ACTIVE)
    Robot::lastTimeActive = millis();
  else if(state == IDDLE)
    Motors::stop();
};

void Robot::setAlarm(RobotAlarm _alarm){
  Robot::alarm = _alarm;

  if(_alarm != NONE){
    LOG("Alarm set: "); LOG(_alarm); LOG("\n");
    threadBeeper.enabled = true;
    Robot::doBeep(5, 30);
    Robot::setState(IDDLE);
  }else{
    LOG("Alarm clear\n");
  }
};


// ====================================
//       BEEP and Voltage states
// ====================================

float Robot::vbat = 0;

int Robot::beepTimes = 2;
int Robot::beepInterval = 50;
BeepState Robot::beepState = BEEP_NONE;

void Robot::setBeep(BeepState state){
  // Skip if already in that state
  if(Robot::beepState == state)
    return;

  Robot::beepState = state;
  threadBeeper.enabled = true;

  LOG("Beep State: "); LOG(state); LOG("\n");
}

void Robot::doBeep(int _times, int interval){
  Robot::beepTimes = _times;
  Robot::beepInterval = interval;

  // Make sure Thread will run
  threadBeeper.enabled = true;

  LOG("Beep times: "); LOG(_times); LOG("\n");
}

void threadBeeper_run(){
  static bool isBeeping = false;
  static bool lastIsBeeping = false;

  // if(Robot::alarm != NONE){
    // High Priority to alarms
    // isBeeping = !isBeeping;
    // threadBeeper.setInterval(isBeeping ? 20 : 80);
  // }else
  if(Robot::beepTimes > 0){
    // Decrease beeps at each beep
    if(!isBeeping)
      Robot::beepTimes--;

    isBeeping = !isBeeping;

    threadBeeper.setInterval(isBeeping ? 30 : Robot::beepInterval);
  }else if(Robot::beepState == BEEP_NONE){
    isBeeping = false;
    threadBeeper.enabled = false;
    threadBeeper.setInterval(0);
  }else if(Robot::beepState == WARN){
    isBeeping = !isBeeping;

    threadBeeper.setInterval(isBeeping ? 30 : 1000);
  }else if(Robot::beepState == ALARM){
    isBeeping = !isBeeping;

    threadBeeper.setInterval(isBeeping ? 30 : 200);
  }

  if(lastIsBeeping != isBeeping){
    digitalWrite(PIN_BUZZER, isBeeping);
    lastIsBeeping = isBeeping;
  }
}


// ====================================
//         ATTITUDE/ORIENTATION
// ====================================

int16_t Robot::dx = 0;
int16_t Robot::dy = 0;
float Robot::theta = 0;
bool Robot::onFloor = false;
bool Robot::inclinated = false;
