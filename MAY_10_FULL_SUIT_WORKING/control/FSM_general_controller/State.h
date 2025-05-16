#ifndef STATE_H
#define STATE_H

#include <iostream>
using namespace std;

class State
{
public:
  State();

  virtual ~State();

  virtual void enter() = 0;
  virtual void update() = 0;
  virtual void leave() = 0;
  static State *current; // Declaration
  static State *active;  // Declaration
  static State *idle;    // Declaration
  static State *error;   // Declaration

  double getMaxSensorReading();
  double getMinSensorReading();
};

#endif