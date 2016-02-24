#include "_config.h"
#include "_types.h"


#ifndef CONTROLLER_H
#define CONTROLLER_H


class Controller{

public:
  // ====================================
  //           INITIALIZATION
  // ====================================

  static void init();


  // ====================================
  //           CONTROLL TARGETS
  // ====================================

  static float targetY;
  static float targetTheta;

  static TargetType targetYType;
  static TargetType targetThetaType;

  static void setTargetY(TargetType type, float val);
  static void setTargetTheta(TargetType type, float val);
  static void setPIDConstants(float kp, float ki, float kd, float iLimit);

};


#endif
