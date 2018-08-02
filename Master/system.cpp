#include "_config.h"
#include "_types.h"

#include "robot.h"
#include "system.h"
#include "attitude.h"

ThreadController controller;

//
// Battery Check specific
//
void threadBatteryChecker_run();
Thread threadBatteryChecker(threadBatteryChecker_run, 3000);

//
// WatchDog LED thread
//
void threadWatchdog_run();
Thread threadWatchdog(threadWatchdog_run, 500);

//
//  Serial Debug thread
//
void threadDebug_run();
Thread threadDebug(threadDebug_run, 20);

float System::dt = 0;

// ====================================
//            INITIALIZATION
// ====================================

void System::init(){

  // Initialize Serial and Wait to be ok
  Serial.begin(SERIAL_SPEED);
  
  LOG("\n===== "); LOG(PROJECT_NAME); LOG(" =====\n");
  LOG(PROJECT_VERSION); LOG("\n\n");

  LOG("System::init\n");

  // Add threads to system
  controller.add(&threadBatteryChecker);
  controller.add(&threadWatchdog);
  // controller.add(&threadDebug);
}

// ====================================
//          THREAD CALLBACKS
// ====================================


// Checks for low battery and set alarm states accordingly
void threadBatteryChecker_run(){
  static int alerts = 0;

  long analogVal = analogRead(PIN_VBAT);
  Robot::vbat = VBAT_VOLTAGE(analogVal);

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

// Watches robot and checks for activity. Also, toggles led
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

}

// // Logs through serial some robot variables
// void threadDebug_run(){
//   LOG("Lin: ");
//   LOG(Robot::linear);
//   LOG("ang: ");
//   LOG(Robot::angular);
//   LOG(" runTime: ");
//   LOG(System::dt);ENDL;
// }
