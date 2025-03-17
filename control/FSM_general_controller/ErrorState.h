#ifndef ERROR_STATE_H
#define ERROR_STATE_H

#include "State.h"

class ErrorState : public State
{

public:
  ErrorState();
  virtual ~ErrorState();
  void enter();
  void leave();
  void update();
};

#endif