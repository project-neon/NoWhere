#include "_config.h"
#include "_types.h"

#include "imu.h"
#include "system.h"
#include "attitude.h"

void attitude_run();

IMUThread *sensorIMU;

void Attitude::init(){

  LOG("Attitude::init\n");

  sensorIMU = new IMUThread();
  sensorIMU->init();
  sensorIMU->setInterval(9);

  system.add(sensorIMU);
}
