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
PID pidThetaRate(0.2f, 2.5f, 0.0f, 40);//, *pidThetaAbs;
// PID *pidYRate, *pidYAbs;

float absoluteY = 0;


// ====================================
//           INITIALIZATION
// ====================================

void Controller::init(){

  LOG("Controller::init\n");

  // Initialize PID's
  // pidThetaAbs = new PID(1.0f, 0.0f, 0.0f, 100);
  // pidThetaRate = new PID(0.01f, 0.001f, 0.0f, 40);

  // pidYAbs = new PID(1.0f, 0.0f, 0.0f, 100);
  // pidYRate = new PID(1.0f, 0.0f, 0.0f, 100);

  system.add(&threadCheckInclinationAlarm);
  system.add(&threadController);
}


// ====================================
//           CONTROLL TARGETS
// ====================================

float Controller::targetY = 0;
float Controller::targetTheta = 0;

TargetType Controller::targetYType = TARGET_NONE;
TargetType Controller::targetThetaType = TARGET_NONE;


void Controller::setTargetY(TargetType type, float val){
  // if(type != Controller::targetYType){
    // pidYAbs->reset();
    // pidYRate->reset();
  // }

  // Reset Y Refference
  absoluteY = 0;

  Controller::targetY = val;
  Controller::targetYType = RATE_OF_CHANGE;
}

void Controller::setTargetTheta(TargetType type, float val){
  // if(type == RATE_OF_CHANGE){
    // pidThetaRate.reset();
  // }

  Controller::targetTheta = val;
  Controller::targetThetaType = RATE_OF_CHANGE;
}

void Controller::setPIDConstants(float kp, float ki, float kd, float iLimit){
  pidThetaRate.kp = kp;
  pidThetaRate.ki = ki;
  pidThetaRate.kd = kd;
  pidThetaRate.iLimit = iLimit;
}


// ====================================
//          THREAD CALLBACKS
// ====================================

void threadCheckInclinationAlarm_run(){
  static int alarms = 0;

  // Check Theta Limit (Pitch Roll)
  if(Robot::state == IDDLE && Robot::alarm == ALARM_TOO_INCLINATED){
    // Check if stable, and reset flag
    if(abs(Robot::ypr[1] * 180/M_PI) < 45 && abs(Robot::ypr[2] * 180/M_PI) < 45){
      Robot::setAlarm(NONE);
      alarms = 0;
    }
  }else if(Robot::state == ACTIVE){
    // Check if is too inclinated
    if(abs(Robot::ypr[1] * 180/M_PI) > 45 || abs(Robot::ypr[2] * 180/M_PI) > 45){
      // if(++alarms > 0){
      Robot::setAlarm(ALARM_TOO_INCLINATED);
      // }
    }
  }


}

int logs = 0;
void threadController_run(){
  static float lastYaw = 0;
  static long lastRun;
  float dt;

  logs++;

  // Compute dt
  long now = millis();
  dt = (now - lastRun) / 1000.0;
  lastRun = now;

  // Compute Degrees/second
  float yaw = Robot::ypr[0] * 180/M_PI;

  // Normalize yaw
  float rate = (yaw - lastYaw);

  if (rate > 180.0)
      rate -= 360.0;
    else if (rate < -180.0)
      rate += 360.0;
  rate = rate / dt;

  // Saves Last Yaw
  lastYaw = yaw;

  // Rate is absurd? Skip this controll.
  if(rate < -360 || rate > 360){
    LOG(" ! Absurd rate ");
    LOG(rate);
    LOG(" Yaw ");
    LOG(yaw);
    LOG("\n");
    return;
  }


  // Checks if robot is in IDDLE state. Skip if so...
  if(Robot::state == IDDLE){
    // Motors::stop();
    return;
  }

  // Skip if dt is too large or too small
  if(dt > 0.1 || dt <= 0.0){
    LOG(" ! Weird dt\n");
    return;
  }

  // Final Speed
  float ySpeed = 0;
  float thetaSpeed = 0;

  if(Controller::targetThetaType == RATE_OF_CHANGE){

    pidThetaRate.setTarget(Controller::targetTheta);
    thetaSpeed = pidThetaRate.update(rate, dt);

    if(logs % 50 == 0){
      LOG(" rate: "); LOG(rate);
      LOG(" targ: "); LOG(Controller::targetTheta);
      // LOG(" dt: "); LOG(dt);
      LOG(" ki: "); LOG(pidThetaRate.ki);
    }
  }else if(Controller::targetThetaType == ABSOLUTE){
    // Get YAW
    // float yaw = Robot::ypr[0] * 180/M_PI;

    // pidThetaRate.setTarget(Controller::targetTheta);
    // thetaSpeed = pidThetaRate.update(yaw, dt);

  }else{}

  if(Controller::targetYType == RATE_OF_CHANGE){
    // Use a simple Constant
    ySpeed = Controller::targetY;

  }else if(Controller::targetThetaType == ABSOLUTE){
    // Integrate speed
    float speed = Motors::avgSpeed / 10.0;

    // Integrate Y
    absoluteY += speed * dt;

    ySpeed = Controller::targetY - absoluteY;
    // pidThetaRate.setTarget(Controller::targetY);
    // ySpeed = pidThetaRate.update(absoluteY, dt);

  }else{}



  float left = ySpeed - thetaSpeed;
  float right = ySpeed + thetaSpeed;

  if(logs % 50 == 0){
    LOG(" PWER: [");
    LOG(ySpeed);
    LOG(", ");
    LOG(thetaSpeed);
    LOG("] ");
    LOG(" y: "); LOG(Robot::ypr[0] * 180/M_PI); LOG("\n");
  }

  Motors::setPower(right, left);

  // LOG(" p: "); LOG(Robot::ypr[1] * 180/M_PI);
  // LOG(" r: "); LOG(Robot::ypr[2] * 180/M_PI);

}
