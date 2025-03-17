#include "ActiveState.h"
#include <iostream>

// Constructor
ActiveState::ActiveState()
{
}

// Destructor
ActiveState::~ActiveState()
{
}

void ActiveState::enter()
{
  cout << "\nNow in Active State \n";
}

void ActiveState::update()
{
  while (true)
  {
    cout << "1. idle \n2. error \n3. exit\n";
    int num;
    cin >> num; // replace with sensor reading(s)
    switch (num)
    {

      // replace cases with conditionals

    case 1:
      leave();
      current = idle;
      return;

    case 2:
      leave();
      current = error;
      return;

    case 3:
      exit(0); // exit program with 0 status (immediate shut down case)
    }
  }
}

void ActiveState::leave()
{
  cout << "\nleaving Active State\n";
}
