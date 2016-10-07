#include <Arduino.h>

// #include "OdometryPacket.h"

#define MSK_START       (0b10101010)
#define FLOAT_SCALE     100.0

// Converts an signed integer to a 7 bits signed integer
uint8_t to7bits(int8_t n){
  // Constrain n
  n = constrain(n, -64, 63);
  // Out = ( no sign bit  )   (    only sign bit    )
  return   (n & 0b00111111) | ((n & 0b10000000) >> 1);
}

// Converts an signed 7 bits integer to a 8 bits integer
int8_t from7bits(uint8_t n){
  // Out = ( no sign bit  )   (    copy sign bit    )
  return   (n & 0b01111111) | ((n & 0b01000000) << 1);
}

// Given the parameters, create a packet and sets the given array data
void makeOdometryPacket(uint8_t frame[9],
  int16_t dx,
  int16_t dy,
  float dt,
  bool inclinated,
  bool onFloor){
    static int16_t _dt;

    // Scale floats
    _dt = dt * FLOAT_SCALE;

    frame[0] = MSK_START; // 0b10000000 is Start mask

    // Inclinated / onFloor
    frame[1] = 0x00 | (onFloor << 0) | (inclinated << 1);

    // Include values
    frame[2] = (dx >> 0);
    frame[3] = (dx >> 8);
    frame[4] = (dy >> 0);
    frame[5] = (dy >> 8);
    frame[6] = (_dt >> 0);
    frame[7] = (_dt >> 8);

    // Checksum (Without MSB bit)
    frame[8] =
      frame[0] ^
      frame[1] ^
      frame[2] ^
      frame[3] ^
      frame[4] ^
      frame[5] ^
      frame[6] ^
      frame[7];
}

// From a existing array, extracts information and sends to parameters.
// Returns true if packet is valid. False if not.
bool readOdometryPacket(uint8_t frame[9],
  int16_t &dx,
  int16_t &dy,
  float &dt,
  bool &inclinated,
  bool &onFloor){
    static int16_t _dt;

    // Check start byte
    if(frame[0] != MSK_START)
      return false;

    // Validate packet
    uint8_t checksum =
      frame[0] ^
      frame[1] ^
      frame[2] ^
      frame[3] ^
      frame[4] ^
      frame[5] ^
      frame[6] ^
      frame[7];

    // Check if checksum matches calculated one
    if(checksum != frame[8])
      return false;

    // Extracts booleans from second byte
    onFloor =    frame[1] & (0x1 << 0);
    inclinated = frame[1] & (0x1 << 1);

    // Extracts values from next bytes
    dx  = frame[2] | (frame[3] << 8);
    dy  = frame[4] | (frame[5] << 8);
    _dt = frame[6] | (frame[7] << 8);

    // Converts to float
    dt = _dt / FLOAT_SCALE;

    return true;
}
