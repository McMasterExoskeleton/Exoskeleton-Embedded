#ifndef IDLE_STATE_H
#define IDLE_STATE_H

#include "State.h"

class IdleState : public State
{

public:
  IdleState();
  virtual ~IdleState();
  void enter();
  void leave();
  void update();
};

#endif