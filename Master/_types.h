#include <stdint.h>
#include <math.h>
//#include <helper_3dmath.h>

#ifndef TYPES_H
#define TYPES_H

enum RobotState{
  IDDLE = 0,
  ACTIVE = 1
};

enum RobotAlarm{
  NONE = 0,
  ALARM_INIT,
  ALARM_LOW_BATTERY,
  ALARM_LOST_CONNECTION,
  ALARM_TOO_INCLINATED
};

enum TargetType{
  TARGET_NONE = 0,
  ABSOLUTE = 1,
  RATE_OF_CHANGE = 2
};

enum BeepState{
  UNDEF = -1,
  BEEP_NONE = 0,
  WARN = 1,
  ALARM = 2
};

#endif
