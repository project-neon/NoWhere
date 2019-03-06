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

RF24 radio(PIN_RADIO1_CE, PIN_RADIO1_CSN);

void configNRF(RF24 &radio);

// Radio buffer
uint8_t radioBufferIn[RADIO_PACKET_SIZE];
uint8_t robotQuantity = 0;
uint8_t robotId = 0;
float robotYSpeed = 0;
float robotTSpeed = 0;
uint8_t myRobotId = 0;

// Checks for communication and sets robot's state to IDDLE/ACTIVE
void threadIddleDetection_run();
Thread threadIddleDetection(threadIddleDetection_run, 100);

// Serial communication (DEBUG)
void threadSerial_run();
Thread threadSerial(threadSerial_run, 50);

// NRF Communication
void threadNRF_run();
Thread threadNRF(threadNRF_run, 1000); // Starts listening after 1sec, then set itself to 0ms

// This robot Address
byte addresses[][6] = {"1Node","2Node"};

// ====================================
//           INITIALIZATION
// ====================================

void Commander::init(){

  LOG("Commander::init"); ENDL;
  delay(10);

  // Initialize Radio
  if(!radio.begin()){
    while(1){
      digitalWrite(12, !digitalRead(12));  // statement
      delay(100);
    }
  }else{
    LOG("Radio Begin OK");ENDL;
  }
  delay(10);

  // Configure Radio
  configNRF(radio);

  // Open Communication Pipes
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1,addresses[0]);
  radio.startListening();

  // Add Commander Threads to controller
  controller.add(&threadIddleDetection);
  controller.add(&threadSerial);
  controller.add(&threadNRF);
}

void configNRF(RF24 &radio){
  radio.powerUp();
  // Try change params, if failed LOG and Halt
  if(!radio.setDataRate(RF24_1MBPS)){
    LOG(" ! Failed to setup Radio");
    ENDL;
    while(1){
      LOG(".");
      digitalWrite(PIN_LED2, !digitalRead(PIN_LED2));
      delay(1000);
    }
  }
  // Set retries to zero to always send new information.
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(Robot::getRobotChannel());
  radio.setRetries(0, 0);
  LOG("Radio::init OK"); ENDL;
}

// ====================================
//    Code to scan 2.4GHz Enviroment.
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
  configNRF(radio);
}


// ====================================
//   Detects Iddle activity on Radio
// ====================================

void threadIddleDetection_run(){
  // Only checks if state of robot is ACTIVE
  if(Robot::state != ACTIVE)
    return;

  // Set state to iddle after radio timeout
  if(millis() - Robot::lastTimeActive > RADIO_TIMEOUT_TO_IDDLE){
    Robot::setState(ACTIVE);
    //Robot::doBeep(2, 50);
  }
}

// ====================================
//   Parses Serial commands from USB
// ====================================

void threadSerial_run(){

  // Wait for variables to be available
  if(!Serial.available()){
    return;
  }
  
  // Reads from serial buffer
  char got = Serial.read();

  LOG("Cmd:"); LOG(got); ENDL;
  
  if(got == '1'){
    Motors::stop();
    Controller::enabled = false;
    Robot::setState(ACTIVE);
    Motors::setPower(20, 0);
  }
  else if(got == '2'){
    Motors::stop();
    Controller::enabled = false;
    Robot::setState(ACTIVE);
    Motors::setPower(-20, 0);
  }
  else if(got == '3'){
    Motors::stop();
    Controller::enabled = false;
    Robot::setState(ACTIVE);
    Motors::setPower(0, 20);
  }
  else if(got == '4'){
    Motors::stop();
    Controller::enabled = false;
    Robot::setState(ACTIVE);
    Motors::setPower(0, -20);
  }
  else if(got == '5'){
    Motors::stop();
    Controller::enabled = false;
    Robot::setState(ACTIVE);
    Motors::setPower(20, 20);
  }
  else if(got == '6'){
    Motors::stop();
    Controller::enabled = false;
    Robot::setState(ACTIVE);
    Motors::setPower(-20, -20);
  }
  else if(got == '0'){

    Controller::enabled = true;
    Controller::setTarget(0, 0, 0);
    Motors::stop();

  }else if(got == 'b'){
    float bat = analogRead(A3);
    LOG("Raw: "); LOG(bat); ENDL;
    LOG("Battery: "); LOG(Robot::vbat); ENDL;

  }else if(got == 'i'){
    delay(1);
    if(Serial.available() > 0){
      char newId = Serial.read();
      if(newId != '\r' && newId != '\n'){
        LOG("Id set! RESTART to change Radio"); ENDL;
        Robot::setRobotID(int(newId)-'0');
      }
    }

    LOG("ID: "); LOG(Robot::getRobotID()); ENDL;
    LOG("[Send i<char> to set robot's id]");
  }else if(got == 'c'){
    String num = "";
    
    if(Serial.available() > 0){
      while(Serial.available() > 0){
        char in = Serial.read();
        num+=in;
      }
    }
      uint8_t newChannel = num.toInt();
      if(newChannel > 0 && newChannel <= 126){
        Robot::setRobotChannel(newChannel);
        configNRF(radio);
        LOG("Channel set! RESTART to change Radio to: "); LOG(newChannel); ENDL;
      }else{
        LOG("ID: "); LOG(Robot::getRobotChannel()); ENDL;
        LOG("[Send c<number between 0 and 126> to set robot's channel]");
      }
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
    LOG("---- help ----"); ENDL;
    LOG("0: Stop motors"); ENDL;
    LOG("1: Left A Motor Front"); ENDL;
    LOG("2: Left A Motor Back"); ENDL;
    LOG("3: Right B Motor Front"); ENDL;
    LOG("4: Right B Motor Back"); ENDL;
    LOG("5: Motors Front"); ENDL;
    LOG("6: Motors Back"); ENDL;
    LOG("d: Enable debug flag"); ENDL;
    LOG("b: Get bat. voltage"); ENDL;
    LOG("i: View robot ID"); ENDL;
    LOG("s: Scan NRF Frequencies"); ENDL;
    LOG("i<char>: Set robot id"); ENDL;
    LOG("c<number from 0 to 126>: Set robot's channel"); ENDL;  
    LOG("@: Replicate odometry Serial"); ENDL;
  }
}

// ====================================
//   CallBack to parse Radio Messages
// ====================================

void threadNRF_run(){

  static bool available;
  static bool activate;

  // Set own interval to 0ms
  threadNRF.setInterval(0);

  available = false;

  memset(radioBufferIn, 0, sizeof(radioBufferIn));

  // While data incoming, read it
  while (radio.available()) {
    available = true;
    radio.read(&radioBufferIn, RADIO_PACKET_SIZE);
  }

  // Skip if no data received
  if(!available)
    return;
  
  // Parse message
  robotQuantity = radioBufferIn[0];

  // Search for my id and data between received message 
  for(int i=0; i < robotQuantity; i++){
    robotId = radioBufferIn[1+(i*ROBOT_PACKET_SIZE)];
    if(robotId == Robot::getRobotID()){
      // If id Found gather other data
      myRobotId = robotId;
      activate = radioBufferIn[2+(i*ROBOT_PACKET_SIZE)];
      robotYSpeed = radioBufferIn[3+(i*ROBOT_PACKET_SIZE)] | (radioBufferIn[4+(i*ROBOT_PACKET_SIZE)] << 8);
      robotTSpeed = radioBufferIn[5+(i*ROBOT_PACKET_SIZE)] | (radioBufferIn[6+(i*ROBOT_PACKET_SIZE)] << 8);
      
      robotYSpeed = robotYSpeed / FLOAT_MULTIPLIER;
      robotTSpeed = robotTSpeed / FLOAT_MULTIPLIER;

      Controller::setTarget(0, robotYSpeed, robotTSpeed);
      radio.startListening();

      // Force Disable robot if said
      if(!activate){
        Robot::setState(IDDLE);
      }

      // Got message. Robot is now Active if message was an action message
      if(Robot::alarm == NONE){
        if(Robot::state == IDDLE && activate){
          Robot::setState(ACTIVE);
          Robot::doBeep(2, 100);
        }
        // Save timestamp of message
        Robot::lastTimeActive = millis();
      }
    }
  }
}
