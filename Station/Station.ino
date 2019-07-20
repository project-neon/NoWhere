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
    - To Set Up the number of Robots you are trying to control type:
      '::s?' ? being the number of robots you wish to comunicate
    - Start and finish always with the token ":", and send as this:
    
      :[TARGET_ROBOT_ID];[STATE];[linearSpeed];[thetaSpeed]:[TARGET_ROBOT2_ID];[STATE];[linearSpeed];[thetaSpeed]:[TARGET_ID];[STATE];[linearSpeed];[thetaSpeed];
      
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
    
      Front Robot reference -> SIDE THAT DON'T HAVE A USB PORT

    - Commands are delimited by '\n'
      
*/

// Make logging Easier
#define LOG                 Serial.print
#define ENDL                LOG("\n")
#define DEBUG               

// Radio Harware Pins
#define PIN_RADIO_CE       7
#define PIN_RADIO_CSN      6
#define PIN_LED             13

// Transmission Stuff
#define ROBOT_PACKET_SIZE   6

#define SEARCH_OFFSET 6

uint8_t sizeDataOut = 0;
byte radioAck = 0;

bool shouldTransmit = true;

#define ROBOT_QUANTITY_ADDRESS 0

// Use the same value at the robot to decode the float values.
#define FLOAT_MULTIPLIER    10.0
#define PID_FLOAT_MULTIPLIER    1000.0

int robotQuantity = 0;

//Modes
static bool PID_Mode = 0;

void handleMessage();

// Radio Objects
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);

//Data Holders
char serialDataIn[64];
float robotBufferIn[2];

// Robot addresses
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
  radio.setChannel(88);
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
}


void setup(){
  // Setup LED
  pinMode(PIN_LED, OUTPUT);

  // Initialize Serial
  Serial.begin(115200);
  while(!Serial);
  Serial.setTimeout(4);

  radio.begin();
  configNRF(radio);
  LOG("start"); ENDL;
  LOG("number of robots:");
  robotQuantity = EEPROM.read(ROBOT_QUANTITY_ADDRESS); 
  LOG(robotQuantity); ENDL;

  
  sizeDataOut = (ROBOT_PACKET_SIZE*robotQuantity)+1;

  #ifdef DEBUG
    sizeDataOut += 8;
  #endif

  // Configure Output Pipe
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  radio.startListening();
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
  
  static bool ack;
  float tmpFloat;
  int endIndex = 0;
  int startIndex = 0;

  uint8_t radioDataOut[sizeDataOut];
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

  // Change mode to PID_Mode
  if(serialDataIn[1] == ':' && serialDataIn[2] == 'p'){
    PID_Mode = !PID_Mode;
    LOG("PID_Mode mode is "); LOG(PID_Mode ? "ON" : "OFF"); ENDL;
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
    uint8_t robotId = tmp.toInt();
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
    uint8_t robotState = tmp.toInt();
    #ifdef DEBUG
      LOG("~robotState: "); LOG(robotState); ENDL;
    #endif

    // if to robot state 3 (pid mode) and parse message to send   :id ; activate(3); linear(0) ou angular(1) ; p ; i ; d: 

    if (PID_Mode && robotState==3){

       ///////////////////////// Read PID_setting

    _startIndex = _endIndex + 1;
    _endIndex = message.indexOf(';', _startIndex);
    if(_endIndex < 0){
      LOG("nok PID_setting\n");
      return;
    }
    tmp = message.substring(_startIndex, _endIndex);
    uint8_t PID_setting = tmp.toInt();
    #ifdef DEBUG
      LOG("~PID_setting: "); LOG(PID_setting); ENDL;
    #endif

      ///////////////////////// Read constP
    _startIndex = _endIndex + 1;
    _endIndex = message.indexOf(';', _startIndex);
    if(_endIndex < 0){
     LOG("nok constP \n"); 
     return;
    }
    tmp = message.substring(_startIndex, _endIndex);
    tmpFloat = tmp.toFloat();
    int16_t constP = tmpFloat * PID_FLOAT_MULTIPLIER ;
    #ifdef DEBUG
      LOG("~constP: "); LOG(constP); ENDL;
    #endif

    ///////////////////////// Read constI
    _startIndex = _endIndex + 1;
    _endIndex = message.indexOf(';', _startIndex);
    if(_endIndex < 0){
     LOG("nok constI \n"); 
     return;
    }
    tmp = message.substring(_startIndex, _endIndex);
    tmpFloat = tmp.toFloat();
    int16_t constI = tmpFloat * PID_FLOAT_MULTIPLIER ;
    #ifdef DEBUG
      LOG("~constI: "); LOG(constI); ENDL;
    #endif

    ///////////////////////// Read constD
    _startIndex = _endIndex + 1;
    _endIndex = message.indexOf(':', _startIndex);
    if(_endIndex < 0){
     LOG("nok constD \n"); 
     return;
    }
    tmp = message.substring(_startIndex, _endIndex);
    tmpFloat = tmp.toFloat();
    int16_t constD = tmpFloat * PID_FLOAT_MULTIPLIER ;
    #ifdef DEBUG
      LOG("~constD: "); LOG(constD); ENDL;
    #endif


    // Number of Robots

    radioDataOut[0] = robotQuantity;

    // ID
    radioDataOut[1+(i*ROBOT_PACKET_SIZE)] = robotId;

    // IDDLE/ACTIVE
    radioDataOut[2+(i*ROBOT_PACKET_SIZE)] = robotState;

    // PID_setting
    radioDataOut[3+(i*ROBOT_PACKET_SIZE)] = PID_setting;
    
    //  constP
    radioDataOut[4+(i*ROBOT_PACKET_SIZE)] = constP & 0xff;
    radioDataOut[5+(i*ROBOT_PACKET_SIZE)] = (constP >> 8) & 0xff;
    radioDataOut[6+(i*ROBOT_PACKET_SIZE)] = (constP >> 16) & 0xff;
    radioDataOut[7+(i*ROBOT_PACKET_SIZE)] = (constP >> 24) & 0xff;

    //  constI
    radioDataOut[8+(i*ROBOT_PACKET_SIZE)] = constI & 0xff;
    radioDataOut[9+(i*ROBOT_PACKET_SIZE)] = (constI >> 8) & 0xff;
    radioDataOut[10+(i*ROBOT_PACKET_SIZE)] = (constI >> 16) & 0xff;
    radioDataOut[11+(i*ROBOT_PACKET_SIZE)] = (constI >> 24) & 0xff;

    //  constD
    radioDataOut[12+(i*ROBOT_PACKET_SIZE)] = constD & 0xff;
    radioDataOut[13+(i*ROBOT_PACKET_SIZE)] = (constD >> 8) & 0xff;
    radioDataOut[14+(i*ROBOT_PACKET_SIZE)] = (constD >> 16) & 0xff;
    radioDataOut[15+(i*ROBOT_PACKET_SIZE)] = (constD >> 24) & 0xff;

    startIndex = endIndex; // Reset startIndex for next robot message

      
    }else{

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
      LOG("~targetTheta: "); LOG(robotTargetT); ENDL;
    #endif


    // Number of Robots

    radioDataOut[0] = robotQuantity;

    // ID
    radioDataOut[1+(i*ROBOT_PACKET_SIZE)] = robotId;

    // IDDLE/ACTIVE
    radioDataOut[2+(i*ROBOT_PACKET_SIZE)] = robotState;
    
    // Y Speed
    radioDataOut[3+(i*ROBOT_PACKET_SIZE)] = robotTargetY & 0xff;
    radioDataOut[4+(i*ROBOT_PACKET_SIZE)] = (robotTargetY >> 8) & 0xff;

    // Theta Speed
    radioDataOut[5+(i*ROBOT_PACKET_SIZE)] = robotTargetT & 0xff;
    radioDataOut[6+(i*ROBOT_PACKET_SIZE)] = (robotTargetT >> 8) & 0xff;

    startIndex = endIndex; // Reset startIndex for next robot message

    }
      
    
  }
  #ifdef DEBUG
    for(int i=0; i < sizeDataOut; i++){
        LOG("Data ");
        LOG(i);
        LOG(" :");
        LOG(radioDataOut[i]);ENDL;
    }
  #endif
  //
  // Send to NRF
  //

  // Validate robotId
  for(int i=0; i < robotQuantity; i++){
    // robotId should be between 1 and 10
    if(!(radioDataOut[1+(i*ROBOT_PACKET_SIZE)] >= 1 && radioDataOut[1+(i*ROBOT_PACKET_SIZE)] <= 10)){
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
  radio.stopListening();
  radio.openWritingPipe(addresses[0]);
  if(shouldTransmit){
    if(!radio.write( &radioDataOut, sizeDataOut)){
      LOG("nok"); ENDL;
    }else{
      LOG("ok"); ENDL;
    } 
  }
  
  radio.startListening();

  // Check if timeout is needed
  while(radio.available()) {
      radio.read(&robotBufferIn, 8);
    }

  
  #ifdef DEBUG
    LOG(" took ");
    LOG(millis() - took); ENDL;

    if(PID_Mode){
      LOG("ErrY: ");
      LOG(robotBufferIn[0]);ENDL;
      LOG("ErrTheta: ");
      LOG(robotBufferIn[1]);ENDL;
    }else{
      LOG("Id: ");
      LOG(robotBufferIn[0]);ENDL;
      LOG("Bat: ");
      LOG(robotBufferIn[1]);ENDL;

    }

  #endif
}
