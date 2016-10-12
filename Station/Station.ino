// System Includes

// External Libraries
#include <SPI.h>
#include <RF24.h>

// Custom Util

// Modules

/*
	Master module that communicates with robots through Radio.

	This program serves as a Slave for the PC, and as Master
	for the robots.

	The API sent from this Master to Serial port is explained above:
	 - Commands are delimited by '\n'
	 - The first character determines the type of the message:
	 	 '~' Is used for DEBUG purposes.
		 '=' Is used to return values
		 '!' Is used to send Error messages (Important)

	The API used when receiving data from Serial is:
	 - Commands are delimited by '\n'
	 - The first character determines the type of action:
	 	 ':' Sends a command to a robot
	 	 	Usage: ':[MSG_ID];[TARGET_ID];[Cmd];[thetaType];[thetaSpeed];[yType];[ySpeed]'
	 	 	Where TARGET_ID is an Integer, And all other are float params;

	 	 	When Message is resolved, it will return:
	 	 		'=[MSG_ID];[RAW_RESPONSE_DATA]\n'


*/

#define LOG                 Serial.print
#define ENDL                LOG("\n")

#define PIN_RADIO1_CE		    9
#define PIN_RADIO1_CSN      10
#define PIN_RADIO2_CE		    6
#define PIN_RADIO2_CSN      7

#define RADIO_PACKET_SIZE   8

#define FLOAT_MULTIPLIER    10.0

#define MASTER_ADDRESS	 	  1
#define ROBOT_ADDRESS		    2

void handleMessage();

// Radio Objects
RF24 radioIn(PIN_RADIO1_CE, PIN_RADIO1_CSN);
RF24 radioOut(PIN_RADIO2_CE, PIN_RADIO2_CSN);

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
  if(!radio.setDataRate(RF24_250KBPS)){
    LOG("! Failed to setup Radio"); ENDL;
  }
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(108);
  radio.setPayloadSize(RADIO_PACKET_SIZE);
  radio.setRetries(1, 1);
  radio.setAutoAck(true);
}


bool sendSpeed(int Channel, int thetaType, float thetaVal, int yType, float yVal);

void setup(){
  // Setup LED
	pinMode(13, OUTPUT);

	// Initialize Serial
	Serial.begin(115200);
	while(!Serial);
  Serial.setTimeout(5);

  // Debug
	Serial.print("~ ----- MasterFirmware -----\n");

	radioIn.begin();
  configNRF(radioIn);
	radioOut.begin();
  configNRF(radioOut);
  Serial.print("~ Radio ok\n");

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
bool debugSend = false;
void loop(){
  static unsigned long lastDebugSend = 0;

  //
  // Step 0: Send Debug message every X ms
  //
  if(debugSend && millis() - lastDebugSend >= 20){
    lastDebugSend = millis();
    char buf[] = "hello!?";
    radioOut.openWritingPipe(addressesOut[1]);
    if(radioOut.write(buf, 8)){
      Serial.println("Sent!");
    }else{
      // Serial.println("Failed!");
    }
  }

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

    // Increment size
    serialDataPos++;

    // Check for end of packet (\n);
    if(in == '\n'){
      // Valid packet only if lower than 63 bytes
      if(serialDataPos < 63){
        newPacket = true;
        serialDataIn[serialDataPos + 1] = '\0';
      }

      serialDataPos = 0;
    }
  }

  //
  // Parse station commands (do not start with ':')
  //
  if(newPacket && serialDataIn[0] == 'd'){
    newPacket = false;
    debugSend = !debugSend;
    LOG(debugSend ? "~Start Transmiting..." : "Stop Transmiting."); ENDL;
  }

  //
  // Step 3: If new packet available, parse and send it
  //
  if(newPacket){
    handleMessage();
  }


  //
  // Step 2: Check incoming data in Receiving NRF
  //
  while(radioIn.available()){
    char buf[8];
    radioIn.read(&buf, 8);
    Serial.print("~Received: ");
    Serial.println(buf);
  }
}

void handleMessage(){
  float tmpFloat;
  int endIndex = 0;
  int startIndex = 0;

  // Convert to String
  message = String(serialDataIn);

  // Check start token
  if(serialDataIn[0] != ':'){
    LOG("!Invalid start token: "); LOG(serialDataIn[0], HEX); ENDL;
    return;
  }

  // Parse Message ID
	startIndex = endIndex + 1;
	endIndex = message.indexOf(';', startIndex);
	if(endIndex < 0){
		LOG("! Missing Message ID\n");
		return;
	}
	tmp = message.substring(startIndex, endIndex);
	int msgId = tmp.toInt();
  // LOG("~msgId: "); LOG(msgId); ENDL;

  // Parse Robot ID
  startIndex = endIndex + 1;
	endIndex = message.indexOf(';', startIndex);
	if(endIndex < 0){
		LOG("! Missing Robot ID\n");
		return;
	}
	tmp = message.substring(startIndex, endIndex);
	int robotId = tmp.toInt();
  // LOG("~robotId: "); LOG(robotId); ENDL;

  // Read Target state
  startIndex = endIndex + 1;
	endIndex = message.indexOf(';', startIndex);
	if(endIndex < 0){
		LOG("! Missing Target State (IDDLE/ACTIVE)\n");
		return;
	}
	tmp = message.substring(startIndex, endIndex);
	int robotState = tmp.toInt();
  // LOG("~robotState: "); LOG(robotState); ENDL;

  // Read target Y Speed
  startIndex = endIndex + 1;
	endIndex = message.indexOf(';', startIndex);
	if(endIndex < 0){
		LOG("! Missing Target Y\n");
		return;
	}
	tmp = message.substring(startIndex, endIndex);
	tmpFloat = tmp.toFloat();
  int16_t robotTargetY = tmpFloat * FLOAT_MULTIPLIER;
  // LOG("~robotTargetY: "); LOG(robotTargetY); ENDL;

  // Read target Y Speed
  startIndex = endIndex + 1;
	endIndex = message.indexOf(';', startIndex);
	if(endIndex < 0){
		LOG("! Missing Target theta\n");
		return;
	}
	tmp = message.substring(startIndex, endIndex);
	tmpFloat = tmp.toFloat();
  int16_t robotTargetT = tmpFloat * FLOAT_MULTIPLIER;
  // LOG("~robotTargetT: "); LOG(robotTargetT); ENDL;


  //
  // Prepares buffer to send data
  //

  // Clear buffer
  memset(radioDataOut, 0, sizeof(radioDataOut));

  // Make data
  radioDataOut[0] = uint8_t(msgId);

  // IDDLE/ACTIVE
  radioDataOut[1] = robotState;

  // Y Speed
  radioDataOut[2] = robotTargetY;
  radioDataOut[3] = robotTargetY >> 8;

  // Theta Speed
  radioDataOut[4] = robotTargetT;
  radioDataOut[5] = robotTargetT >> 8;


  //
  // Send to NRF
  //

  // Validate robotId
  if(robotId < 1 || robotId > 3){
    LOG("! Invalid robot ID\n");
		return;
  }

  // Select destination
  radioOut.openWritingPipe(addressesOut[robotId]);
  if(radioOut.write(radioDataOut, 8)){
    LOG("="); LOG(msgId); LOG(";"); LOG("ok"); ENDL;
  }else{
    LOG("="); LOG(msgId); LOG(";"); LOG("err"); ENDL;
  }


}
