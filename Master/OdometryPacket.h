#ifndef ODOMETRYPACKET_H
#define ODOMETRYPACKET_H

 #include <stdint.h>

extern void makeOdometryPacket(uint8_t frame[5],
  float dx,
  float dy,
  float dt,
  bool inclinated,
  bool onFloor);

extern bool readOdometryPacket(uint8_t frame[5],
  float &dx,
  float &dy,
  float &dt,
  bool &inclinated,
  bool &onFloor);

#endif
