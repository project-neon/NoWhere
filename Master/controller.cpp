#include <Thread.h>

#include "_config.h"
#include "_types.h"

#include "pid.h"
#include "robot.h"
#include "system.h"
#include "motors.h"
#include "controller.h"

//
// Local variables/objects
//
void threadController_run();
Thread threadController(threadController_run, 10);

void threadCheckInclinationAlarm_run();
Thread threadCheckInclinationAlarm(threadCheckInclinationAlarm_run, 500);

//
// PID's
//
PID pidX(0.2f, 2.5f, 0.0f, 40);
PID pidY(0.2f, 2.5f, 0.0f, 40);
PID pidTheta(0.2f, 2.5f, 0.0f, 40);


// ====================================
//           INITIALIZATION
// ====================================

void Controller::init(){
  LOG("Controller::init\n");

  system.add(&threadCheckInclinationAlarm);
  system.add(&threadController);
}


// ====================================
//           CONTROLL TARGETS
// ====================================

float Controller::targetY = 0;
float Controller::targetX = 0;
float Controller::targetTheta = 0;

void Controller::setTarget(float targetX, float targetY, float targetTheta){
  Controller::targetX = targetX;
  Controller::targetY = targetY;
  Controller::targetTheta = targetTheta;
}

void Controller::setPIDConstants(float kp, float ki, float kd, float iLimit){
  pidTheta.kp = kp;
  pidTheta.ki = ki;
  pidTheta.kd = kd;
  pidTheta.iLimit = iLimit;
}


// ====================================
//          THREAD CALLBACKS
// ====================================

void threadCheckInclinationAlarm_run(){
  static int alarms = 0;

  // Check Theta Limit (Pitch Roll)
  if(Robot::state == IDDLE && Robot::alarm == ALARM_TOO_INCLINATED){
    // Check if stable, and reset flag
    if(!Robot::inclinated){
      Robot::setAlarm(NONE);
      alarms = 0;
    }
  }else if(Robot::state == ACTIVE){
    // Check if is too inclinated
    if(Robot::inclinated){
      Robot::setAlarm(ALARM_TOO_INCLINATED);
    }
  }
}

int logs = 0;
void threadController_run(){
  static long lastRun;
  float dt;

  logs++;

  // Compute dt
  long now = millis();
  dt = (now - lastRun) / 1000.0;
  lastRun = now;

  // Skip if dt is too large or too small
  if(dt > 0.1 || dt <= 0.0){
    LOG(" ! Weird dt\n");
    return;
  }

  // Compute Degrees/second
  float rate = Robot::dt;

  // Rate is absurd? Skip this controll.
  if(rate < -360 || rate > 360){
    LOG(" ! Absurd rate "); LOG(rate); LOG("\n");
    return;
  }

  // Checks if robot is in IDDLE state. Skip if so...
  if(Robot::state == IDDLE){
    Motors::stop();
    return;
  }


  // Final Speed
  float speedY = 0;
  float speedTheta = 0;

  pidTheta.setTarget(Controller::targetTheta);
  speedTheta = pidTheta.update(rate, dt);

  float left = speedY - speedTheta;
  float right = speedY + speedTheta;

  // Log once in a while
  if(logs % 50 == 0){
    LOG("    x: "); LOG(Controller::targetX);
    LOG("    y: "); LOG(Controller::targetY);
    LOG(" thet: "); LOG(Controller::targetTheta);
    LOG("\n");
  }

  Motors::setPower(right, left);
}
