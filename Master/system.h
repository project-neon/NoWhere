#include <Thread.h>
#include <ThreadController.h>

#ifndef SYSTEM_H
#define SYSTEM_H

extern ThreadController controller;

class System{
public:
  static void init();
  static float dt;
};

#endif
