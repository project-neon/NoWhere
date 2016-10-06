#include <SPI.h>
#include <Wire.h>
#include <avr/pgmspace.h>

#define PIN_LED             5
#define PIN_MOUSE_NCS       10

#define INTERRUPT_MOUSE     0
#define INTERRUPT_IMU       1

#define LOOP_TIME           5
#define SQUAL_THRESHOLD_UP  10
#define SQUAL_FILTER_LIMIT  10

// Mouse Flags
long x;
long y;

// Debug purposes
bool DEBUG = false;
bool MEASURE = false;
long measured = 0;

// Forward declaration
void haltLED(int n);
bool isOnFloor();
bool isInclinated();

void setup(){
  // Initialize Serial
  Serial.begin(115200);

  // Initialize Pins
  pinMode(PIN_LED, OUTPUT);

  // Initialize SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(8);

  // Initialize I2C
  Wire.begin();
  TWBR = ((16000000L / 400000L) - 16) / 2;
  delay(1);

  // Initialize Mouse
  if(!Mouse_init(PIN_MOUSE_NCS))
    haltLED(2);

  // Link mouse interrupt to method
  // attachInterrupt(0, onMouseMove, FALLING);

  // Initialize IMU
  if(!IMU_init())
    haltLED(3);

  // Link IMU Interrupt to method
  attachInterrupt(INTERRUPT_IMU, onIMURead, RISING);
}

char in;
int statusLedLoopCount;
bool statusLed = false;
unsigned long lastRun = 0;
void loop(){
  // Check for command from Serial port
  if(Serial.available()){
    in = Serial.read();

    if(in == 'd'){
      DEBUG = !DEBUG;
      if(DEBUG) Serial.println("DEBUG=true");
    }else if(in == 'i'){
      // Show mouse info
      Mouse_debug();
    }else if(in == 'm'){
      // Measure enable and reset
      if(!MEASURE){
        measured = 0;
      }
      MEASURE = !MEASURE;
    }
  }

  // Synchronize loop
  while(millis() - lastRun < LOOP_TIME){
    // Keep reading IMU
    IMU_read();
  }
  lastRun = millis();

  // Read mouse motion
  int16_t dx = 0;
  int16_t dy = 0;
  Mouse_readXY(dx, dy);

  // Read IMU
  float dt = 0;
  // TODO

  // Check if it's on the floor
  bool onFloor = isOnFloor();

  // Check Inclination
  bool inclinated = isInclinated();

  // Sends raw data if not in DEBUG mode
  if(!DEBUG){
    // Create empty frame
    uint8_t frame[5];

    // Make up packet with data
    makeOdometryPacket(frame, dx, dy, dt, inclinated, onFloor);

    // Send to serial
    Serial.write(frame, 5);
    // Serial.print(frame[0], BIN);
    // Serial.print(" ");
    // Serial.print(frame[1], BIN);
    // Serial.print(" ");
    // Serial.print(frame[2], BIN);
    // Serial.print(" ");
    // Serial.print(frame[3], BIN);
    // Serial.print(" ");
    // Serial.println(frame[4], BIN);
  }

  // Shows a human readable string if debug mode
  if(DEBUG){
    // Serial.print(MouseMoved);
    // Serial.write('\t');
    Serial.print(dx);
    Serial.write('\t');
    Serial.print(dy);
    Serial.write('\t');
    Serial.print(getYPR(0));
    Serial.write('\t');
    Serial.print(getYPR(1));
    Serial.write('\t');
    Serial.print(getYPR(2));
    // Serial.write('\t');
    // Serial.print(millis() - lastRun);
    Serial.print("\r\n");
  }

  if(MEASURE){
    measured += dy;
    Serial.print(dy);
    Serial.write('\t');
    Serial.print(measured);
    Serial.print("\r\n");
  }

  // Invert LED Status
  if(++statusLedLoopCount > (onFloor ? 10 : 50)){
    statusLedLoopCount = 0;
    statusLed = !statusLed;
    digitalWrite(PIN_LED, statusLed);
  }
}

// Check if it's too much inclinated
bool isInclinated(){
  return false;
}

// Check if it's on the floor
bool isOnFloor(){
  static int onFloorFilter = 0;
  int squals = Mouse_readSqual();

  // If on floor, subtract average, or sum it
  if(squals > SQUAL_THRESHOLD_UP)
    onFloorFilter = min(onFloorFilter + 1, SQUAL_FILTER_LIMIT);
  else
    onFloorFilter = max(onFloorFilter - 1, 0);

  return (onFloorFilter > 0);
}

// Halts the board and keeps blinking an number of times and waiting
void haltLED(int n){
  // Blink n times and wait for 1.5 secconds
  while(true){
    for(int i = 0; i < n; i++){
      digitalWrite(PIN_LED, HIGH);
      delay(60);
      digitalWrite(PIN_LED, LOW);
      delay(100);
    }
    delay(1500);
  }
}
