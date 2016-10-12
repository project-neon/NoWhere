#include <Thread.h>

#include "_config.h"
#include "_types.h"

#include "pid.h"
#include "robot.h"
#include "system.h"
#include "motors.h"
#include "attitude.h"
#include "controller.h"

//
// Local variables/objects
//
void threadController_run();
Thread threadController(threadController_run, 0);

void threadCheckInclinationAlarm_run();
Thread threadCheckInclinationAlarm(threadCheckInclinationAlarm_run, 500);

//
// PID's
//
PID pidX(0.2f, 2.5f, 0.0f, 100);
PID pidY(7.0f, 0.0f, 0.0f, 0);
PID pidTheta(1.0f, 0.0f, 0.02f, 0);

void resetControl();


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
float pwrLeft;
float pwrRight;
void threadController_run(){
  static unsigned long lastNow;
  static unsigned long now;
  static float lastRate;
  static float rateAvgThetha;
  static float rateThetha;
  static float rateSpeed;
  static float dt;
  static bool wasOnFloor = true;

  // Checks if new data is available for processing
  if(!Attitude::newData)
    return;

  // Lower Flag
  Attitude::newData = false;

  // Compute dt
  now = micros();
  dt = (now - lastNow) / 1000000.0;
  lastNow = now;

  // Skip if dt is too large or too small
  if(dt > 0.1 || dt <= 0.0){
    LOG(" ! Weird dt\n");
    return;
  }

  // Compute Degrees/second
  // angulo = Robot::theta
  // dt = dt
  rateThetha = (Robot::theta - lastRate);
  if (rateThetha > 180.0)
    rateThetha -= 360.0;
  else if (rateThetha < -180.0)
    rateThetha += 360.0;
  rateThetha = rateThetha / dt;
  lastRate = Robot::theta;

  // Rate is absurd? Skip this controll.
  if(rateThetha < -1000 || rateThetha > 1000){
    LOG(" ! theta "); LOG(rateThetha); ENDL;
    return;
  }

  // Compute Y Speed rate
  rateSpeed = Robot::dy / dt / 1516.0;

  // rateAvgThetha += (rateThetha - rateAvgThetha) * 0.3;

  // Check if robot is not touching ground
  if(!Robot::onFloor){
    resetControl();
    if(wasOnFloor)
      Robot::doBeep(1, 80);

    wasOnFloor = false;

    return;
  }else if(!wasOnFloor){
    Robot::doBeep(2, 80);
    wasOnFloor = true;
  }

  // Checks if robot is in IDDLE state. Skip if so...
  if(Robot::state == IDDLE){
     resetControl();
     return;
  }

  // Calculate
  // float errorTheta = Controller::targetTheta - rateThetha;
  pidTheta.setTarget(Controller::targetTheta);
  pidY.setTarget(Controller::targetY);

  float speedTheta = pidTheta.update(rateThetha, dt);
  float speedY = pidY.update(rateSpeed, dt);

  // Final Speed
  float accLeft  = speedY + speedTheta;
  float accRight = speedY - speedTheta;

  pwrLeft  = pwrLeft  + accLeft * dt;
  pwrRight = pwrRight + accRight * dt;

  // Limit Pwers
  pwrLeft  = min(100, max(-100, pwrLeft));
  pwrRight = min(100, max(-100, pwrRight));

  Motors::setPower(pwrLeft, pwrRight);

  float errY = pow(Controller::targetY - rateSpeed, 2);
  float errTheta = pow(Controller::targetTheta - rateThetha, 2);

  // Log if debug is enabled
  if(Robot::debug){
    LOG("\tdt: "); LOG(dt * 1000);
    LOG("\terrY: "); LOG(errY);
    LOG("\terrT: "); LOG(errTheta);
    // LOG("\tthet: "); LOG(rateThetha);
    // LOG("\tthet: "); LOG(rateSpeed);
    // LOG("\tx: "); LOG(Controller::targetX);
    // LOG("\ty: "); LOG(Controller::targetY);
    // LOG("\tthet: "); LOG(Controller::targetTheta);
    LOG("\r\n");
  }

}

// Resets the PID and stop motors
void resetControl(){
  Motors::stop();
  pidY.reset();
  pidX.reset();
  pidTheta.reset();
  pwrLeft = 0;
  pwrRight = 0;
}
