#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 mpu;

// IMU state variables
bool newImuData;
float ypr[3];
Quaternion q;
VectorFloat gravity;

float getYPR(int n){
  return ypr[n];
}

// Global flag that must be set to `true` when interrupt occurs
bool imuInterrupted = false;

void onIMURead(){
    imuInterrupted = true;
}

bool IMU_init(){
  bool devStatus;

  mpu.initialize();

  if(!mpu.testConnection()){
    Serial.println("MPU6050 connection failed");
    return false;
  }

  mpu.setDMPEnabled(false);
  mpu.resetDMP();
  // Reset IMU
  mpu.reset();
  delay(50);

  // Activate MPU6050
  mpu.setSleepEnabled(false);

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(83);
  mpu.setYGyroOffset(-3);
  mpu.setZGyroOffset(48);
  mpu.setZAccelOffset(1974); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

  // Enable Digital Motion Procesing
  mpu.setDMPEnabled(true);

  return true;
}

int took;
int outOfSyncs;
int interruptStatus;
uint8_t packet[64];
bool IMU_read(){
  static int fifoCount = 0;

  // Check for interrupt
  if(!imuInterrupted && fifoCount < 42)
    return false;

  // Clear interrupt flag
  imuInterrupted = false;

	// Check for new Packet
  delayMicroseconds(500);
	interruptStatus = mpu.getIntStatus();
  delayMicroseconds(50);
	fifoCount = mpu.getFIFOCount();

  if ((interruptStatus & 0x10) || fifoCount == 1024 || fifoCount % 42 > 0) {
    // reset so we can continue cleanly
    delayMicroseconds(50);
    mpu.resetFIFO();
    delayMicroseconds(50);
    fifoCount = 0;
    return false;
  }

  // Check if new data available
  bool run = (interruptStatus & 0x02);

  // Don't read if not ready
  if(!run)
    return false;

  unsigned long start = millis();

	// read a packet from FIFO
  delayMicroseconds(50);
	mpu.getFIFOBytes(packet, 42);
  fifoCount -= 42;

	// Convert and save state to Object
	mpu.dmpGetQuaternion(&q, packet);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

	newImuData = true;

  took = millis() - start;
}
