#include "_config.h"
#include "_types.h"

#ifndef ROBOT_H
#define ROBOT_H


class Robot{
public:


  // ====================================
  //           INITIALIZERS
  // ====================================

  static void init();


  // ====================================
  //   SPECIFIC CONFIGURATIONS (EEPROM)
  // ====================================

  static uint8_t getRobotID();
  static void setRobotID(uint8_t id);
  static uint8_t getRobotChannel();
  static void setRobotChannel(uint8_t channel);

  // ====================================
  //         PRIMITIVE STATES
  // ====================================

  static bool debug;
  /*
    Boolean flag indicating Active stabilizing/Power on motors
  */
  static RobotState state;

  /*
    Last time when robot was Active
  */
  static unsigned long lastTimeActive;

  /*
    Current alarm of the robot (Errors)
  */
  static RobotAlarm alarm;

  /*
    Set's the state to the robot
  */
  static void setState(RobotState _state);

  /*
    Set's the current alarm of the robot, and put's robbot on Iddle
  */
  static void setAlarm(RobotAlarm _alarm);


  // ====================================
  //       BEEP and Voltage states
  // ====================================

  /*
    Battery voltage (in volts, ohhh!)
  */
  static float vbat;

  /*
    Beeper flag/setter
  */
  static uint8_t beepTimes;
  static uint8_t beepInterval;
  static BeepState beepState;

  static void setBeep(BeepState state);
  static void doBeep(uint8_t _times, uint8_t interval, uint8_t _reason);


  // ====================================
  //         ATTITUDE/ORIENTATION
  // ====================================

  static int16_t dx;
  static int16_t dy;
  static float theta;
  static float angular;
  static float linear;
  static bool onFloor;
  static bool inclinated;

};

#endif
