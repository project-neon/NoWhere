#include <Arduino.h>

// #include "OdometryPacket.h"

#define MSK_START       (0b10000000)
#define VALUE_SCALE     2.0

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
void makeOdometryPacket(uint8_t frame[5],
  int dx,
  int dy,
  int dt,
  bool inclinated,
  bool onFloor){

    frame[0] = MSK_START; // 0b1XXXXXXX is Start mask
    frame[0]|= onFloor    << 0;
    frame[0]|= inclinated << 1;

    // Include values
    frame[1] = to7bits(int8_t(dx));
    frame[2] = to7bits(int8_t(dy));
    frame[3] = to7bits(int8_t(dt));

    // Checksum (Without MSB bit)
    frame[4] = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3]) & 0b01111111;
}

// From a existing array, extracts information and sends to parameters.
// Returns true if packet is valid. False if not.
bool readOdometryPacket(uint8_t frame[5],
  int &dx,
  int &dy,
  int &dt,
  bool &inclinated,
  bool &onFloor){

    // Validate packet
    uint8_t checksum = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3]) & 0b01111111;

    // Check if checksum matches calculated one
    if(checksum != frame[4])
      return false;

    // Check starting bits
    if(!(frame[0] & MSK_START) ||
        (frame[1] & MSK_START) ||
        (frame[2] & MSK_START) ||
        (frame[3] & MSK_START) ||
        (frame[4] & MSK_START))
      return false;

    // Extracts booleans from first byte
    onFloor =    frame[0] & (0x1 << 0);
    inclinated = frame[0] & (0x1 << 1);

    // Extracts values from next bytes
    dx = from7bits(frame[1]);
    dy = from7bits(frame[2]);
    dt = from7bits(frame[3]);

    return true;
}
