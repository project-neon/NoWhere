// External Libraries
#include <SPI.h>
#include <RF24.h>

/*
 * 
  Project Neon 2017
  Created by: MarceloFariaz

	Station that communicates with robots through Radio,
  by Joystick Control.
  
*/

// Make logging Easier
#define LOG                 Serial.print
#define ENDL                LOG("\n")

// Radio Hardware Pins
#define PIN_RADIO1_CE        6
#define PIN_RADIO1_CSN      7
#define PIN_LED             5

// Joystick Hardware Pins
#define PIN_JOYSTICK_VCC A0
#define PIN_JOYSTICK_VER A1
#define PIN_JOYSTICK_HOR A2
#define PIN_JOYSTICK_SEL A3
#define PIN_JOYSTICK_GND A4

// Transmission Stuff
#define RADIO_PACKET_SIZE   8
#define MASTER_ADDRESS	 	  1
#define ROBOT_ADDRESS		    2

// Define the max speeds 
#define ROBOT_MAX_Y   60.0 * 10.0
#define ROBOT_MAX_T   600.0 * 10.0

// Radio Object
RF24 radioOut(PIN_RADIO1_CE, PIN_RADIO1_CSN);

//Data Holder
uint8_t radioDataOut[RADIO_PACKET_SIZE];

// Robot Addresses
int robotId = 1;
byte addressesOut[][6] = {
  "",
  "robt1",
  "robt2",
  "robt3"
};

// Configures Speed/Channel/... in Radio
void configNRF(RF24 &radio){
  // Configure Radio
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
  radio.setCRCLength(RF24_CRC_8);
  radio.setPayloadSize(RADIO_PACKET_SIZE);
  radio.setRetries(1, 1);
  radio.setAutoAck(true);
}


bool sendSpeed(int Channel, int thetaType, float thetaVal, int yType, float yVal);

void setup(){
  // Setup LED
  pinMode(PIN_LED, OUTPUT);

  // Setup Joystick
  pinMode(PIN_JOYSTICK_VCC, OUTPUT);
  digitalWrite(PIN_JOYSTICK_VCC, HIGH);
  pinMode(PIN_JOYSTICK_VER, INPUT);
  pinMode(PIN_JOYSTICK_HOR, INPUT);
  pinMode(PIN_JOYSTICK_SEL, INPUT_PULLUP);
  pinMode(PIN_JOYSTICK_GND, OUTPUT);
  digitalWrite(PIN_JOYSTICK_GND, LOW);

  // Initialize Serial
  Serial.begin(115200);
  Serial.print("Joystick Connected!\n");
  
  radioOut.begin();
  configNRF(radioOut);
  Serial.print("start\n");

  // Configure Output Pipe
  radioOut.openWritingPipe(addressesOut[1]);
  radioOut.stopListening();
}

void loop(){

  //
  // Read and convert the joystick controls
  //
  bool select = !digitalRead(PIN_JOYSTICK_SEL);

  if (select) {
    do {
      delay(50);
      select = !digitalRead(PIN_JOYSTICK_SEL);
    } while (select);

    robotId ++;

    if (robotId > 3) 
      robotId = 1;

    LOG("Robot ID: ");
    LOG(robotId);
    ENDL;
  }
  
  int vertical = analogRead(PIN_JOYSTICK_VER);
  int horizontal = analogRead(PIN_JOYSTICK_HOR);

  vertical = map(vertical, 0, 1023, 0, 200) - 100;
  horizontal = map(horizontal, 0, 1023, 0, 200) - 100;

  vertical = vertical > 5 ||  vertical < -5 ? vertical : 0;
  horizontal = horizontal > 5 || horizontal < -5 ? horizontal : 0;

  int robotState = 1;
  int16_t robotTargetY = ROBOT_MAX_Y * (vertical / 100.0);
  int16_t robotTargetT = ROBOT_MAX_T * (horizontal / 100.0);

  LOG(robotTargetY);
  LOG("\t");
  LOG(robotTargetT);
  ENDL;
  
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
    //LOG("ok"); ENDL;
    digitalWrite(PIN_LED, HIGH);
  }else{
    //LOG("nok"); ENDL;
    if(digitalRead(PIN_LED)){
      digitalWrite(PIN_LED, LOW);  
    }
  }
}
