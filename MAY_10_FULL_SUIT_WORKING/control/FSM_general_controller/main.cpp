#include <iostream>
#include <stdlib.h>
#include "ActiveState.h"
#include "IdleState.h"
#include "ErrorState.h"
#include "State.h"
#include "Sensor.h"

using namespace std;

State *State::active = new ActiveState();
State *State::idle = new IdleState();
State *State::error = new ErrorState();

// starting state
State *State::current = State::idle;

int main(int argc, char **argv)
{
  double max_sensor_reading = 1000; // accurate sensor values to be determined
  double min_sensor_reading = -1000;

  /*

  cout << "\n\nInput maximum sensor reading (currently not used for state transitions)\n";
  cin >> max_sensor_reading;
  cout << "\n\n";
  cout << "Input minimum sensor reading (currently not used for state transitions)\n";
  cin >> min_sensor_reading;
  cout << "\n\n";

  */

  cout << "Type 1 or 2 to change states (to exit program type 3) \n\n";

  // Set the max and min sensor readings in the Sensor class
  Sensor::setMaxSensorReading(max_sensor_reading);
  Sensor::setMinSensorReading(min_sensor_reading);

  while (true)
  {
    State::current->enter();
    State::current->update();
  }

  return 0;
}
