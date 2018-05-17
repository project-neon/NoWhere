#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>

/*
  Project Neon 2017
  Created by: Project Neon
  Modified by: Ivan Seidel, João Pedro Vilas

  Basic Radio programming for receiving and parsing the messages 
  received over radio.

    - The message will start with the number of robots messages 
    sent by the transmitter and then be like this for the number 
    of robots previously stated.
      
      - [ROBOT_ID]
      - [STATE]
      - [linearSpeed]
      - [linearSpeed >> 8]
      - [thetaSpeed]
      - [thetaSpeed >> 8]
      
      ROBOT_ID -> INTEGER of the robots ID, can be found by 
      connecting the robot through it's micro USB, and sending 
      'i' over it's Serial connection. If the ROBOT_ID is equal 
      to the one on this robot, the 5 following data belong to 
      this robot.
      
      STATE -> BOOLEAN of the Robot's STATE, either ACTIVE{1} 
      or IDLE{0}. If the robot is active it will move as the 
      command sent.

      LinearSpeed -> FLOAT, robot's linear speed in cm/s

      thetaSpeed -> FLOAT, robot's angular speed in degrees/s    
*/

// Make logging Easier
#define LOG                 Serial.print
#define ENDL                LOG("\n")
#define DEBUG

// Radio Harware Pins
#define PIN_RADIO1_CE       7
#define PIN_RADIO1_CSN      8
#define PIN_LED             13

#define RADIO_PACKET_SIZE   32
#define ROBOT_PACKET_SIZE   6
uint8_t radioBufferIn[RADIO_PACKET_SIZE];

// Use the same value at the station to decode the float values.
#define FLOAT_MULTIPLIER    10.0

// Just for test purposes
#define ROBOT_ID 2

char* receivedMessage = "ok?";
char* notOnMessage = "nok?";

// Radio Objects
RF24 radioIn(PIN_RADIO1_CE, PIN_RADIO1_CSN);

// Robot Addresses
byte addresses[][6] = {"1Node","2Node"};

// Configures Speed/Channel/... in Radio
void configNRF(RF24 &radio){
  
  // Configure Radio
  radio.powerUp();
  if(!radio.setDataRate(RF24_1MBPS)){
    LOG("nok init"); 
    ENDL;
    while(1){
      LOG(".");
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(1000);
    }
  }
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(108);
  radio.setRetries(0, 1);

  #ifdef DEBUG
    LOG("Radio Configured with settings: "); ENDL;
    LOG("Radio Channel: ");
    LOG(radio.getChannel()); ENDL;
    LOG("Radio PA Level: ");
    LOG(radio.getPALevel()); ENDL;
    LOG("Radio Data Rate: ");
    LOG(radio.getDataRate()); ENDL;
    LOG("Radio CRC Lenght: ");
    LOG(radio.getCRCLength()); ENDL;
  #endif

  radioIn.openWritingPipe(addresses[1]);
  radioIn.openReadingPipe(1,addresses[0]);
  radioIn.startListening();
}

void setup(){

  // Setup LED
  pinMode(PIN_LED, OUTPUT);

  // Initialize Serial
  Serial.begin(115200);
  while(!Serial);
  radioIn.begin();
  configNRF(radioIn);
  LOG("start"); ENDL;

}

void loop(){
  static bool available;
  static bool gotMyPackage;
  available = false;
  gotMyPackage = false;

  while (radioIn.available()){                                // While there is data ready
    available = true;
    radioIn.read( &radioBufferIn, RADIO_PACKET_SIZE);             // Get the payload
  }

  if(!available)
    return;

  uint8_t robotQuantity = radioBufferIn[0];
  uint8_t robotId = 0;
  bool active = 0;
  int16_t robotYSpeed = 0;
  int16_t robotTSpeed = 0;
  uint8_t myRobotId = 0;

  for(int i=0; i < robotQuantity; i++){
    robotId = radioBufferIn[1+(i*ROBOT_PACKET_SIZE)];
    if(robotId == ROBOT_ID){
      myRobotId = robotId;
      active = radioBufferIn[2+(i*ROBOT_PACKET_SIZE)];
      robotYSpeed = radioBufferIn[3+(i*ROBOT_PACKET_SIZE)] | (radioBufferIn[4+(i*ROBOT_PACKET_SIZE)] << 8);
      robotTSpeed = radioBufferIn[5+(i*ROBOT_PACKET_SIZE)] | (radioBufferIn[6+(i*ROBOT_PACKET_SIZE)] << 8);
      radioIn.stopListening();
      LOG("receivedMessage: ");
      LOG(myRobotId);ENDL;
      radioIn.openWritingPipe(addresses[1]);
      radioIn.write( myRobotId, sizeof(byte) );              // Send the final one back.      
      radioIn.startListening();  
      #ifdef DEBUG
        LOG("Found my ID among those ");
        LOG(robotQuantity); 
        LOG(" robots!");ENDL; 
        LOG("robotId: ");
        LOG(robotId); ENDL;
        LOG("active: ");
        LOG(active); ENDL;
        LOG("robotYSpeed: ");
        LOG(robotYSpeed); ENDL;
        LOG("robotTSpeed: ");
        LOG(robotTSpeed); ENDL;
      #endif
    }  
  }
}