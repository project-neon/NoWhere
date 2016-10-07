#ifndef ODOMETRYPACKET_H
#define ODOMETRYPACKET_H

 #include <stdint.h>

extern void makeOdometryPacket(uint8_t frame[9],
  int16_t dx,
  int16_t dy,
  float dt,
  bool inclinated,
  bool onFloor);

extern bool readOdometryPacket(uint8_t frame[9],
  int16_t &dx,
  int16_t &dy,
  float &dt,
  bool &inclinated,
  bool &onFloor);

#endif
