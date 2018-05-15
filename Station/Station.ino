// External Libraries
#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>
/*

  Project Neon 2017
  Created by: Project Neon
  Modified by: Ivan Seidel, João Pedro Vilas

	Station that communicates with robots through Radio,
  by parsing and retransmitting the messages sent over serial.

	This program serves as a Slave for the PC, and as Master
	for the robots.

  The API to send commands to Robots is described below:
    - Start always with the token ":", and send as this:
    
      :[TARGET_ROBOT1_ID];[STATE];[linearSpeed];[thetaSpeed]:[TARGET_ROBOT2_ID];[STATE];[linearSpeed];[thetaSpeed]:[TARGET_ID];[STATE];[linearSpeed];[thetaSpeed];
      
      or:
      
      :1;1;100;100:2;1;-100;-100:3;1;300;300:
      
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
// #define DEBUG

// Radio Harware Pins
#define PIN_RADIO1_CE       7
#define PIN_RADIO1_CSN      6
#define PIN_LED             13

// Transmission Stuff
#define RADIO_PACKET_SIZE   6
#define SEARCH_OFFSET 6
uint8_t sizeDataOut = 0;
bool shouldTransmit=true;

#define ROBOT_QUANTITY_ADDRESS 0

// Use the same value at the robot to decode the float values.
#define FLOAT_MULTIPLIER    10.0

int robotQuantity = 0;

void handleMessage();

// Radio Objects
RF24 radioOut(PIN_RADIO1_CE, PIN_RADIO1_CSN);

//Data Holders
char serialDataIn[64];

// Robot Addresses
byte addresses[][6] = {"outb1","inbo1"};

// Configures Speed/Channel/... in Radio
void configNRF(RF24 &radio){
  
  // Configure Radio
  radio.powerUp();
  if(!radio.setDataRate(RF24_250KBPS)){
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
  radio.setRetries(0, 0);
  radio.enableDynamicAck();

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
}


void setup(){
  // Setup LED
  pinMode(PIN_LED, OUTPUT);

  // Initialize Serial
  Serial.begin(115200);
  while(!Serial);
  Serial.setTimeout(4);

  radioOut.begin();
  configNRF(radioOut);
  LOG("start"); ENDL;
  LOG("number of robots:");
  robotQuantity = EEPROM.read(ROBOT_QUANTITY_ADDRESS); 
  LOG(robotQuantity); ENDL;
  
  sizeDataOut = RADIO_PACKET_SIZE*robotQuantity;

  // Configure Output Pipe
  radioOut.openWritingPipe(addresses[0]);
  radioOut.openReadingPipe(1,addresses[1]);
  radioOut.startListening();
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
      serialDataPos = 0;
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
      }else{
        LOG("nok morethan_64");ENDL;
        serialDataPos = 0;
      }
    }
  }

  //
  // Step 2: If new packet available, parse and send it
  //
  
  if(newPacket){
    handleMessage();
  }
}

void handleMessage(){ 

  #ifdef DEBUG
    unsigned long took = millis();
  #endif

  float tmpFloat;
  int endIndex = 0;
  int startIndex = 0;

  int16_t radioDataOut[sizeDataOut];
  memset(radioDataOut, 0, sizeof(radioDataOut));

  // Convert to String
  message = String(serialDataIn);

  // Check start token
  if(serialDataIn[0] != ':'){
    LOG("nok message start token not found"); LOG(serialDataIn[0], HEX); ENDL;
    return;
  }

  // Parse Second item
  if(serialDataIn[1] == ':' && serialDataIn[2] == 's'){
    EEPROM.write(ROBOT_QUANTITY_ADDRESS,int(serialDataIn[3])-'0');
    Serial.print("number of robots set to: ");
    Serial.println(int(serialDataIn[3])-'0');
    Serial.println("!!!!!! Reset so changes take effect !!!!!!");
    return;
  }

  
  for(int i=0; i < robotQuantity; i++){
    startIndex = message.indexOf(':', startIndex);
    endIndex = message.indexOf(':', startIndex + SEARCH_OFFSET);
    String robotMessage = message.substring(startIndex, endIndex);
    #ifdef DEBUG
      LOG(robotMessage);ENDL;
    #endif
    int _startIndex = 0; 
    int _endIndex = 0;

    //////////////////////// Parse Robot ID

    _startIndex = startIndex + 1;
    _endIndex = message.indexOf(';', _startIndex);
    if(_endIndex < 0){
      LOG("nok id not found\n");
      return;
    }
    tmp = message.substring(_startIndex, _endIndex);
    int robotId = tmp.toInt();
    #ifdef DEBUG
      LOG("~robotId: "); LOG(robotId); ENDL;
    #endif

    ///////////////////////// Read Target state

    _startIndex = _endIndex + 1;
    _endIndex = message.indexOf(';', _startIndex);
    if(_endIndex < 0){
      LOG("nok state not found\n");
      return;
    }
    tmp = message.substring(_startIndex, _endIndex);
    int robotState = tmp.toInt();
    #ifdef DEBUG
      LOG("~robotState: "); LOG(robotState); ENDL;
    #endif

    ///////////////////////// Read target Linear(Y) Speed
    _startIndex = _endIndex + 1;
    _endIndex = message.indexOf(';', _startIndex);
    if(_endIndex < 0){
     LOG("nok ySpeed \n"); 
     return;
    }
    tmp = message.substring(_startIndex, _endIndex);
    tmpFloat = tmp.toFloat();
    int16_t robotTargetY = tmpFloat * FLOAT_MULTIPLIER;
    #ifdef DEBUG
      LOG("~robotTargetY: "); LOG(robotTargetY); ENDL;
    #endif

    ///////////////////////// Read target Theta Speed
    _startIndex = _endIndex + 1;
    _endIndex = message.indexOf(':', _startIndex);
    if(endIndex < 0){
      LOG("nok thetaSpeed\n");
      return;
    }
    tmp = message.substring(_startIndex, _endIndex);
    tmpFloat = tmp.toFloat();
    int16_t robotTargetT = tmpFloat * FLOAT_MULTIPLIER;
    #ifdef DEBUG
      LOG("targetTheta: "); LOG(robotTargetT); ENDL;
    #endif

    // ID
    radioDataOut[0+(i*RADIO_PACKET_SIZE)] = robotId;

    // IDDLE/ACTIVE
    radioDataOut[1+(i*RADIO_PACKET_SIZE)] = robotState;
    
    // Y Speed
    radioDataOut[2+(i*RADIO_PACKET_SIZE)] = robotTargetY;
    radioDataOut[3+(i*RADIO_PACKET_SIZE)] = robotTargetY >> 8;

    // Theta Speed
    radioDataOut[4+(i*RADIO_PACKET_SIZE)] = robotTargetT;
    radioDataOut[5+(i*RADIO_PACKET_SIZE)] = robotTargetT >> 8;
    
    startIndex = endIndex; // Reset startIndex for next robot message
    
  }

  //
  // Send to NRF
  //

  // Validate robotId
  for(int i=0; i < robotQuantity; i++){
    // robotId should be between 1 and 10
    if(!(radioDataOut[0+(i*RADIO_PACKET_SIZE)] >= 1 && radioDataOut[0+(i*RADIO_PACKET_SIZE)] <= 10)){
        LOG("Wrong radioId ");
        LOG("message: ");
        LOG(i+1); ENDL;
        shouldTransmit = false;   // Do not transmit this time
        break;
    }
    else
      shouldTransmit = true;    // Everything alrigth
  }

  // Select destination
  radioOut.stopListening();
  radioOut.openWritingPipe(addresses[0]);
  if(shouldTransmit){
    if(!radioOut.write(radioDataOut, RADIO_PACKET_SIZE,1)){
      LOG("nok");
      if(digitalRead(PIN_LED)){
        digitalWrite(PIN_LED, LOW);  
      }
      #ifndef DEBUG
        ENDL;
      #endif
    }else{
      LOG("ok");
      digitalWrite(PIN_LED, HIGH);
      #ifndef DEBUG
        ENDL;
      #endif
    } 
  }
  radioOut.startListening();
  #ifdef DEBUG
    LOG(" took ");
    LOG(millis() - took); ENDL;
  #endif
}
