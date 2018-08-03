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

int logs = 0;
float pwrLeft;
float pwrRight;
float errY;
float errTheta; 

bool Controller::enabled = true;

void threadController_run();
Thread threadController(threadController_run, 0);

//
// PID's
//

//  PIDs Parameters. 
PID pidY(-10.0f, 0.0f, 0.00f, 100);
PID pidTheta(1.0f, 0.1f, 0.00f, 0);

void resetControl();

// ====================================
//           INITIALIZATION
// ====================================

void Controller::init(){
  LOG("Controller::init\n");
  controller.add(&threadController);
}

// ====================================
//           CONTROLL TARGETS
// ====================================

float Controller::targetY = 0;
float Controller::targetX = 0;
float Controller::targetTheta = 0;

// Function to set targets
void Controller::setTarget(float targetX, float targetY, float targetTheta){
  Controller::targetX = targetX;
  Controller::targetY = targetY;
  Controller::targetTheta = targetTheta;
}

// Function to set Constants
void Controller::setPIDConstants(float kp, float ki, float kd, float iLimit){
  pidTheta.kp = kp;
  pidTheta.ki = ki;
  pidTheta.kd = kd;
  pidTheta.iLimit = iLimit;
}

// Resets the PID and stop motors
void resetControl(){
  Motors::stop();
  pidY.reset();
  pidTheta.reset();
  pwrLeft = 0;
  pwrRight = 0;
}

// ====================================
//          THREAD CALLBACKS
// ====================================

void threadController_run(){
  static unsigned long lastNow;
  static unsigned long now;
  static float lastRate;
  static float rateTheta;
  static float rateSpeed;
  static float dt;
  static bool wasOnFloor = true;

  // Checks if new data is available for processing
  if(!Attitude::newData)
    return;

   // Checks if new data is available for processing
  if(!Controller::enabled){
    threadController.enabled = false;
    return;
  }

  // Lower Flag
  Attitude::newData = false;

  // Compute dt
  now = micros();

  // Bring dt back to seconds
  dt = (now - lastNow) / 1000000.0;
  System::dt = dt;
  lastNow = now;

  // Skip if dt is too large or too small
  if(dt > 0.03 || dt <= 0.00){
    LOG(" ! Weird dt:");
    LOG(dt);  ENDL;
    return;
  }

  // Compute rate of Theta (degrees/s)
  rateTheta = (Robot::theta - lastRate);
  if (rateTheta > 180.0)
    rateTheta -= 360.0;
  else if (rateTheta < -180.0)
    rateTheta += 360.0;
  rateTheta = rateTheta / dt;
  lastRate = Robot::theta;

  Robot::angular = rateTheta;
  
  // Rate is absurd? Skip this controll.
  if(rateTheta < -1200 || rateTheta > 1200){
    LOG(" ! theta "); 
    return;
  }

  // Compute Y Speed rate for 400 Dots Per Inch and some empirical params ((400 * 2.54)+someting)
  rateSpeed = Robot::dy / dt / 1516.0;

  Robot::linear = rateSpeed;

  // Check if robot is not touching ground
  if(!Robot::onFloor){
    resetControl();
    if(wasOnFloor)
      Robot::doBeep(1, 80, 0);

      wasOnFloor = false;

    return;
  }else if(!wasOnFloor){
    Robot::doBeep(Robot::getRobotID(), 80, 1);
    wasOnFloor = true;
  }

  // Checks if robot is in IDDLE state. Skip if so...
  if(Robot::state == IDDLE){
     resetControl();
     return;
  }

  // Set target for PIDs
  pidTheta.setTarget(Controller::targetTheta);
  pidY.setTarget(Controller::targetY);

  // Calculate PID values
  float speedTheta = pidTheta.update(rateTheta, dt);
  float speedY = pidY.update(rateSpeed, dt);

  // Final Speed for robot
  float accLeft  = speedY + speedTheta;
  float accRight = speedY - speedTheta;

  pwrLeft  = pwrLeft  + accLeft * dt;
  pwrRight = pwrRight + accRight * dt;

  // Limit Powers
  pwrLeft  = min(100, max(-100, pwrLeft));
  pwrRight = min(100, max(-100, pwrRight));

  Motors::setPower(pwrLeft, pwrRight);
  
  errY = pow(Controller::targetY - rateSpeed, 2);
  errTheta = pow(Controller::targetTheta - rateTheta, 2);

  // Log if debug is enabled
  if(Robot::debug){
    LOG("\tdt: "); LOG(dt * 1000);
    LOG("\terrY: "); LOG(errY);
    LOG("\terrT: "); LOG(errTheta);
    ENDL;
  }
}
