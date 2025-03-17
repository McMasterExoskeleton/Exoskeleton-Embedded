#include "ErrorState.h"

ErrorState::ErrorState()
{
}

ErrorState::~ErrorState()
{
}

void ErrorState::enter()
{
  cout << "\nNow in Error State \n";
}

void ErrorState::update()
{
  // loop
  while (true)
  {
    cout << "1. active\n2. idle \n3. exit\n";
    int sensorReading;
    cin >> sensorReading; // replace with sensor reading(s)
    switch (sensorReading)
    {

      // replace cases with conditionals

    case 1:
      leave();
      current = active;
      return;

    case 2:
      leave();
      current = idle;
      return;

    case 3:
      exit(0); // exit program with 0 status (immediate shut down case)
    }
  }
}

void ErrorState::leave()
{
  cout << "\nleaving Error State \n";
}
