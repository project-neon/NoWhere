#include "_config.h"
#include "_types.h"

#include "robot.h"
#include "system.h"

ThreadController controller;

//
// Battery Check specific
//
void threadBatteryChecker_run();
Thread threadBatteryChecker(threadBatteryChecker_run, 3000);

//
// WatchDog and LED thread
//
void threadWatchdog_run();
Thread threadWatchdog(threadWatchdog_run, 500);


void System::init(){

  // Initialize Serial and Wait to be ok
/*
  Serial.begin(SERIAL_SPEED);
  while(!Serial);
  delay(50);
*/
  LOG("\n===== "); LOG(PROJECT_NAME); LOG(" =====\n");
  LOG(PROJECT_VERSION); LOG("\n\n");

  LOG("System::init\n");

  // Add threads to system
  controller.add(&threadBatteryChecker);
  controller.add(&threadWatchdog);
}

/*
  Checks battery voltage
*/
void threadBatteryChecker_run(){
  static int alerts = 0;

  long analogVal = analogRead(PIN_VBAT);
  Robot::vbat = VBAT_VOLTAGE(analogVal);
  // LOG("VBAT: "); LOG(Robot::vbat); LOG(" ["); LOG(analogVal); LOG("]\n");

  if(Robot::vbat < VBAT_ALARMED && Robot::vbat > VBAT_USB){
    if(++alerts == 3){
      Robot::setAlarm(ALARM_LOW_BATTERY);
      Robot::setBeep(ALARM);
    }
  }else if(Robot::vbat < VBAT_WARNED && Robot::vbat > VBAT_USB){
    if(alerts <= 0)
      Robot::setBeep(WARN);
  }else{
    // Reset filter
    alerts = 0;
    Robot::setBeep(BEEP_NONE);
  }
}

/*
  Watches role robot and checks for activity. Also, toggles led
*/
void threadWatchdog_run(){
  static bool ledState = false;
  static bool ledStateInvert = false;

  // Toggle state
  ledState = !ledState;
  ledStateInvert = false;

  // Check for ALARM
  // if(Robot::alarm != NONE)
    // ledState = true;
  if(Robot::alarm == NONE && Robot::state == IDDLE){
    // State is iddle without error
    threadWatchdog.setInterval(ledState ? 250 : 750);
  }else if(Robot::alarm == NONE && Robot::state == ACTIVE){
    // State is active (communicating) and not error
    threadWatchdog.setInterval(ledState ? 750 : 250);
  }else{
    threadWatchdog.setInterval(ledState ? 250 : 250);
    ledStateInvert = true;
  }

  // Toggles Led
  digitalWrite(PIN_LED1, ledState);
  digitalWrite(PIN_LED2, ledState ^ ledStateInvert);

  // Set timeout acordingly to Robot's state
  // threadWatchdog.setInterval(Robot::state == IDDLE ? 250 : 500);
}
