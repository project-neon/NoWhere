
#include "_config.h"
#include "_types.h"

#ifndef COMMANDER_H
#define COMMANDER_H

class Commander{
public:


  // ====================================
  //           INITIALIZATION
  // ====================================

  static void init();


  // ====================================
  //           MESSAGE PARSING
  // ====================================

  static bool handleMessage(uint8_t message[], uint8_t len);

  // static long lastPacketTimestamp;
};



#endif
