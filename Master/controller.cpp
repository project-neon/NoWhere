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
//float errY;
//float errTheta;
float Controller::errY;
float Controller::errTheta; 

bool Controller::enabled = true;

void threadController_run();
Thread threadController(threadController_run, 2);

//
// PID's
//

//  PIDs Parameters. 
PID pidY(6.5f, 0.0f, 0.02f, 0);
PID pidTheta(0.5f, 0.005f, 0.003f, 5000);

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
  static float lastTheta = 0;
  static float rateTheta = 0;
  static float rateSpeed = 0;
  static float dt = 0;
  static bool wasOnFloor = true;

  // Checks if new data is available for processing
  if(!Attitude::newData){
    return;
  }
    
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
  dt = (float)(now - lastNow) / 1000000.0f;
  lastNow = now;

  // Skip if dt is too large or too small
  if(dt > 0.1 || dt <= 0.00){
    LOG(" ! Weird dt:");LOG(dt);  ENDL;
    return;
  }


  // Compute rate of Theta (degrees/s)
  rateTheta = (Robot::theta - lastTheta);
  
  if (rateTheta > 180.0f)
    rateTheta -= 360.0f;
  else if (rateTheta < -180.0f)
    rateTheta += 360.0f;

  rateTheta = rateTheta / dt;

  lastTheta = Robot::theta;

  Robot::angular = rateTheta;

  
  // Rate is absurd? Skip this controll.
  if(rateTheta < -1200 || rateTheta > 1200){
    LOG(" ! theta: "); LOG(dt); ENDL;
    return;
  }

  // Compute Y Speed rate for 400 Dots Per Inch and some empirical params ((400 * 2.54)+someting)
  rateSpeed = Robot::dy / dt / 1516.0f;

  Robot::linear = rateSpeed;

  // Check if robot is not touching ground
  if(!Robot::onFloor){
    resetControl();
    if(wasOnFloor)
      Robot::doBeep(1, 80);

      wasOnFloor = false;

    return;
  }else if(!wasOnFloor){
    Robot::doBeep(Robot::getRobotID(), 80);
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
  
  Controller::errY = pow(Controller::targetY - rateSpeed, 2);
  Controller::errTheta = pow(Controller::targetTheta - rateTheta, 2);

}
