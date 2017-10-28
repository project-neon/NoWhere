// External Libraries
#include <SPI.h>
#include <RF24.h>

/*

  Project Neon 2017
  Created by: Ivan Seidel
  Modified by: João Pedro Vilas, and...

	Station that communicates with robots through Radio,
  by parsing and retransmitting the messages sent over serial.

	This program serves as a Slave for the PC, and as Master
	for the robots.

  The API to send commands to Robots is described below:
    - Start always with the token ":", and send as this:
    
      :[TARGET_ID];[STATE];[linearSpeed];[thetaSpeed]
      
      TARGET_ID -> INTEGER of the robots ID, can be found 
      by connecting the robot through it's micro USB, and 
      sending i over it's Serial connection.
      
      STATE -> BOOLEAN of the Robot's STATE, either ACTIVE{1} 
      or IDLE{0}. If the robot is active it will move as the 
      command sent.

      LinearSpeed -> FLOAT, robot's linear speed in cm/s

      thetaSpeed -> FLOAT, robot's angular speed in degrees/s
	 
    - Commands are delimited by '\n'
      
*/


// Make logging Easier
#define LOG                 Serial.print
#define ENDL                LOG("\n")

// Radio Harware Pins
#define PIN_RADIO1_CE		    9
#define PIN_RADIO1_CSN      10
#define PIN_RADIO2_CE		    6
#define PIN_RADIO2_CSN      7

// Transmission Stuff
#define RADIO_PACKET_SIZE   8
#define MASTER_ADDRESS	 	  1
#define ROBOT_ADDRESS		    2

// Use the same value at the robot to decode the float values.
#define FLOAT_MULTIPLIER    10.0

void handleMessage();

// Radio Objects
RF24 radioIn(PIN_RADIO1_CE, PIN_RADIO1_CSN);
RF24 radioOut(PIN_RADIO2_CE, PIN_RADIO2_CSN);

//Data Holders
uint8_t radioDataIn[RADIO_PACKET_SIZE];
uint8_t radioDataOut[RADIO_PACKET_SIZE];
char serialDataIn[64];

// Robot Addresses
byte addressesOut[][6] = {
  "",
  "robt1",
  "robt2",
  "robt3"
};

byte addressesIn[][6] = {
  "",
  "stat1",
  "stat2",
  "stat3"
};

// Configures Speed/Channel/... in Radio
void configNRF(RF24 &radio){
  // Configure Radio
  if(!radio.setDataRate(RF24_1MBPS)){
    LOG("nok init"); 
    ENDL;
    while(1){
      digitalWrite(13, !digitalRead(13));
      delay(200);
    }
  }
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(108);
  radio.setCRCLength(RF24_CRC_8);
  radio.setPayloadSize(RADIO_PACKET_SIZE);
  radio.setRetries(1, 1);
  radio.setAutoAck(true);
}


bool sendSpeed(int Channel, int thetaType, float thetaVal, int yType, float yVal);

void setup(){
  // Setup LED
  pinMode(13, OUTPUT);

  // Initialize Serial
  Serial.begin(500000);
  while(!Serial);
  Serial.setTimeout(5);


  radioIn.begin();
  configNRF(radioIn);
  radioOut.begin();
  configNRF(radioOut);
  Serial.print("start\n");
  // Configure Receiving pipes
  radioIn.stopListening();
  radioIn.openReadingPipe(1, addressesIn[1]); // Robot 1
  radioIn.openReadingPipe(2, addressesIn[2]); // Robot 2
  radioIn.openReadingPipe(3, addressesIn[3]); // Robot 3
  radioIn.startListening();

  // Configure Output Pipe
  radioOut.openWritingPipe(addressesOut[1]);
  radioOut.stopListening();
}


//
// Get's a Delimited Token in the message
//
int getToken(int start, String &in, String &out){
  int endIndex = in.indexOf(';', start);
  if(endIndex < 0){
    return endIndex;
  }
  out = in.substring(start, endIndex);
  return endIndex;
}

String message, tmp;
int serialDataPos = 0;

void loop(){

  //
  // Step 1: Check incoming data from Serial
  //
  char in;
  
  bool newPacket = false;

  while(Serial.available()){
    
    in = Serial.read();
    // Do not append if packet size is greater than 64
    if(serialDataPos < 64)
      serialDataIn[serialDataPos] = in;
    else{
      LOG("nok buf_full");ENDL;
      break;
    }
    // Increment size
    serialDataPos++;

    // Check for end of packet (\n);
    if(in == '\n'){
      // Valid packet only if lower than 63 bytes
      if(serialDataPos < 63){
        newPacket = true;
        serialDataIn[serialDataPos + 1] = '\0';
        serialDataPos = 0;
        break;
      }

    }
  }

  //
  // Step 2: If new packet available, parse and send it
  //
  if(newPacket){
    handleMessage();
  }


  //
  // Step 3: Check incoming data in Receiving NRF
  //
  // while(radioIn.available()){
  //   char buf[8];
  //   radioIn.read(&buf, 8);
  //   //Serial.print("ok ");
  //   //LOG(buf);ENDL;
  // }
}

void handleMessage(){ 
  float tmpFloat;
  int endIndex = 0;
  int startIndex = 0;

  // Convert to String
  message = String(serialDataIn);

  // Check start token
  if(serialDataIn[0] != ':'){
    LOG("nok token not found"); LOG(serialDataIn[0], HEX); ENDL;
    return;
  }
  // Parse Robot ID
  startIndex = endIndex + 1;
	endIndex = message.indexOf(';', startIndex);
	if(endIndex < 0){
		LOG("nok id not found\n");
		return;
	}
	tmp = message.substring(startIndex, endIndex);
	int robotId = tmp.toInt();
  // LOG("~robotId: "); LOG(robotId); ENDL;

  // Read Target state
  startIndex = endIndex + 1;
	endIndex = message.indexOf(';', startIndex);
	if(endIndex < 0){
		LOG("nok state not found\n");
		return;
	}
	tmp = message.substring(startIndex, endIndex);
	int robotState = tmp.toInt();
  // LOG("~robotState: "); LOG(robotState); ENDL;

  // Read target Y Speed
  startIndex = endIndex + 1;
	endIndex = message.indexOf(';', startIndex);
	if(endIndex < 0){
		LOG("nok ySpeed \n");
		return;
	}
	tmp = message.substring(startIndex, endIndex);
	tmpFloat = tmp.toFloat();
  int16_t robotTargetY = tmpFloat * FLOAT_MULTIPLIER;
  // LOG("~robotTargetY: "); LOG(robotTargetY); ENDL;

  // Read target Y Speed
  startIndex = endIndex + 1;
	endIndex = startIndex + 2;
	// if(endIndex < 0){
	// 	LOG("nok thetaSpeed\n");
	// 	return;
	// }
	tmp = message.substring(startIndex, endIndex);
	tmpFloat = tmp.toFloat();
  //LOG("targetTheta: "); LOG(tmpFloat); ENDL;
  int16_t robotTargetT = tmpFloat * FLOAT_MULTIPLIER;


  //
  // Prepares buffer to send data
  //

  // Clear buffer
  memset(radioDataOut, 0, sizeof(radioDataOut));

  // IDDLE/ACTIVE
  radioDataOut[0] = robotState;

  // Y Speed
  radioDataOut[1] = robotTargetY;
  radioDataOut[2] = robotTargetY >> 8;

  // Theta Speed
  radioDataOut[3] = robotTargetT;
  radioDataOut[4] = robotTargetT >> 8;


  //
  // Send to NRF
  //

  // Validate robotId
  if(robotId < 1 || robotId > 6){
    LOG("nok id\n");
		return;
  }

  // Select destination
  radioOut.openWritingPipe(addressesOut[robotId]);
  if(radioOut.write(radioDataOut, 8)){
    LOG("ok"); ENDL;
  }else{
    LOG("nok"); ENDL;
  }
}
