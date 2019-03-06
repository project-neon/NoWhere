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

  static float targetX;
  static float targetY;
  static float targetTheta;
  static float errY;
  static float errTheta;
  
  static bool enabled;
  static void scanErrors();
  static void setTarget(float targetX, float targetY, float targetTheta);
  static void setPIDConstants(float kp, float ki, float kd, float iLimit);
};


#endif
