#ifndef ACTIVE_STATE_H
#define ACTIVE_STATE_H

#include "State.h"
#include "Sensor.h"

class ActiveState : public State
{

public:
  ActiveState();
  virtual ~ActiveState();
  void enter();
  void leave();
  void update();
};

#endif