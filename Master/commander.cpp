#include <SPI.h>
#include <RF24.h>
#include <Thread.h>
#include <Arduino.h>

#include "_config.h"
#include "_types.h"

#include "robot.h"
#include "system.h"
#include "motors.h"
#include "commander.h"
#include "controller.h"

//
// NRF24L01 Config
//
RF24 radio(PIN_RADIO2_CE, PIN_RADIO2_CSN);

void configNRF();
uint8_t radioBufferIn[RADIO_PACKET_SIZE];
uint8_t radioBufferOut[RADIO_PACKET_SIZE];

// Checks for communication and sets robot's state to IDDLE/ACTIVE
void threadIddleDetection_run();
Thread threadIddleDetection(threadIddleDetection_run, 1000);

// Serial communication (DEBUG)
void threadSerial_run();
Thread threadSerial(threadSerial_run, 50);

// NRF Communication
void threadNRF_run();
Thread threadNRF(threadNRF_run, 1000); // Starts listening after 1sec, then set itself to 0ms


// This robot Address
byte ADDRESS_ROBOT[6]            = "robt?";
byte ADDRESS_STATION[6]          = "stat?";

// ====================================
//           INITIALIZATION
// ====================================

void Commander::init(){

  LOG("Commander::init"); ENDL;
  delay(10);

  // Initialize Radio
  LOG("Radio::init"); ENDL;
  delay(10);
  radio.begin();
  LOG("Radio::init OK"); ENDL;
  delay(10);

  // Change ADDRESS_ROBOT to match with Robot's id (from eeprom)
  ADDRESS_ROBOT[4]   = Robot::getRobotID();
  ADDRESS_STATION[4] = Robot::getRobotID();

  // Configure Radio
  configNRF();

  // Listen to it's own ID
  radio.openReadingPipe(1, ADDRESS_ROBOT);
  radio.openWritingPipe(ADDRESS_STATION);

  radio.startListening();

  controller.add(&threadIddleDetection);
  controller.add(&threadSerial);
  controller.add(&threadNRF);
}

void configNRF(){
  // Configure Radio
  if(!radio.setDataRate(RF24_1MBPS)){
    Robot::doBeep(10, 10);
    LOG(" ! Failed to setup Radio"); ENDL;
  }
  radio.setPALevel(RF24_PA_MAX);
  radio.setCRCLength(RF24_CRC_8);
  radio.setChannel(108);
  radio.setPayloadSize(RADIO_PACKET_SIZE);
  radio.setRetries(0, 0);
  radio.setAutoAck(true);
  radio.startListening();
}

// ====================================
// Does a sweep scan on the radio Freq.
// ====================================

const int num_reps = 100;
const uint8_t num_channels = 126;
uint8_t values[num_channels];
void Commander::scanRadio(){
  radio.setAutoAck(false);

  // Get into standby mode
  radio.startListening();
  radio.stopListening();

  // Print out header, high then low digit
  int i = 0;
  while ( i < num_channels ){
    Serial.print(i>>4, HEX);
    ++i;
  }
  Serial.println();
  i = 0;
  while ( i < num_channels ){
    Serial.print(i&0xf, HEX);
    ++i;
  }
  Serial.println();

  // Do Scan sweep
  // Clear measurement values
  memset(values, 0, sizeof(values));

  // Scan all channels num_reps times
  int rep_counter = num_reps;
  while (rep_counter--) {
    int i = num_channels;
    digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));
    while (i--) {
      // Select this channel
      radio.setChannel(i);

      // Listen for a little
      radio.startListening();
      delayMicroseconds(225);

      // Did we get a carrier?
      if ( radio.testCarrier() ){
        ++values[i];
        digitalWrite(PIN_LED2, true);
      }else{
        digitalWrite(PIN_LED2, false);
      }
      radio.stopListening();
    }
  }

  // Print out channel measurements, clamped to a single hex digit
  i = 0;
  while ( i < num_channels ) {
    Serial.print(min(0xf,values[i]&0xf), HEX);
    ++i;
  }
  Serial.println();

  // Roll back modifications
  configNRF();
}


// ====================================
//   Detects Iddle activity on Radio
// ====================================

void threadIddleDetection_run(){
  // Only checks if state of robot is not ACTIVE
  if(Robot::state != ACTIVE)
    return;

  // if(millis() - Robot::lastTimeActive > RADIO_TIMEOUT_TO_IDDLE){
  //   Robot::setState(IDDLE);
  //   Robot::doBeep(2, 50);
  // }
}


// ====================================
//   Parses Serial commands from USB
// ====================================

void threadSerial_run(){
  if(!Serial.available())
    return;

  char got = Serial.read();

  ENDL;
  LOG("Cmd:"); LOG(got); ENDL;

  if(got == '1')
    Motors::setPower(100, 0);

  else if(got == '2')
    Motors::setPower(-100, 0);

  else if(got == '3')
    Motors::setPower(0, 100);

  else if(got == '4')
    Motors::setPower(0, -100);

  else if(got == '8'){
    Controller::setTarget(0, 50, 90);
  }else if(got == '0'){
    Controller::setTarget(0, 0, 0);
    Motors::stop();
  }else if(got == 'b'){
    LOG("Battery: "); LOG(Robot::vbat); ENDL;

  }else if(got == 'i'){
    delay(1);
    if(Serial.available()){
      char newId = Serial.read();
      if(newId != '\r' && newId != '\n'){
        LOG("Id set! RESTART to change Radio"); ENDL;
        Robot::setRobotID(newId);
      }
    }

    LOG("ID: "); LOG(Robot::getRobotID()); ENDL;
    LOG("[Send i<char> to set robot's id]");
  }else if(got == '@'){
    Motors::stop();
    LOG(F("===========================")); ENDL;
    LOG(F("= Serial Interactive Mode =")); ENDL;
    LOG(F("= Send  ':q'  to exit     =")); ENDL;
    LOG(F("===========================")); ENDL;

    bool notExited = true;
    char lastChr = ' ';
    char chr;
    while(notExited){
      // Pipe incoming data from USB Serial to Slave Serial
      while(Serial.available()){
        chr = Serial.read();
        if(chr == 'q' && lastChr == ':')
          notExited = false;
        lastChr = chr;
        Serial1.write(chr);
      }

      // Pipe incoming data from Slave Serial to USB Serial
      while(Serial1.available())
        Serial.write(Serial1.read());
    }
  }else if(got == 'd'){
    Robot::debug = !Robot::debug;
    LOG("Debug mode is "); LOG(Robot::debug ? "ON" : "OFF"); ENDL;
  }else if(got == 's'){
    Motors::stop();
    Commander::scanRadio();
  }else if(got == 'h'){
    ENDL;
    LOG("---- help ----");
    ENDL;
    LOG("0: Stop motors"); ENDL;
    LOG("d: Enable debug flag"); ENDL;
    LOG("b: Get bat. voltage"); ENDL;
    LOG("i: View robot ID"); ENDL;
    LOG("s: Scan NRF"); ENDL;
    LOG("i<char>: Set robot id"); ENDL;
    LOG("@: Replicate odometry Serial"); ENDL;
  }
}


// ====================================
// Parses Incoming Messages from Radio
// ====================================

void threadNRF_run(){
  static bool available;
  static bool activate;
  long start = millis();

  // Set interval to 0ms
  threadNRF.setInterval(0);

  available = false;
  while (radio.available()) {
    available = true;
    radio.read( &radioBufferIn, RADIO_PACKET_SIZE);
    LOG("Rec packet: "); LOG((char*) radioBufferIn); ENDL;
  }

  // Skip if no data received
  if(!available)
    return;

  // Parse message
  activate = radioBufferIn[0];

  int16_t _targetY     = radioBufferIn[1] | (radioBufferIn[2] << 8);
  int16_t _targetTheta = radioBufferIn[3] | (radioBufferIn[4] << 8);

  float targetY     =     _targetY / FLOAT_MULTIPLIER;
  float targetTheta = _targetTheta / FLOAT_MULTIPLIER;

  Controller::setTarget(0, targetY, targetTheta);

  // Build callback message
  radioBufferOut[0] = radioBufferIn[0];
  radioBufferOut[1] = Robot::state;
  radioBufferOut[2] = Robot::vbat * 10;

  // Send message
  radio.stopListening();
  radio.write(&radioBufferOut, 8);
  radio.startListening();

  // Got message. Robot is now Active if message was an action message
  if(Robot::alarm == NONE){
    if(Robot::state == IDDLE && activate){
      Robot::setState(ACTIVE);
      Robot::doBeep(1, 100);
    }

    // Save timestamp of message
    Robot::lastTimeActive = millis();
  }

  // Force Disable robot if said
  if(!activate){
    Robot::setState(ACTIVE);
  }
}
