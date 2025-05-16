#include "IdleState.h"

IdleState::IdleState()
{
}

IdleState::~IdleState()
{
}

void IdleState::enter()
{
  cout << "\nNow in Idle State \n";
}

void IdleState::update()
{
  while (true)
  {

    cout << "1. active\n2. error \n3. exit\n";
    int num;
    cin >> num; // replace with sensor reading(s)
    switch (num)
    {

      // replace cases with conditionals

    case 1:
      leave();
      current = active;
      return;

    case 2:
      leave();
      current = error;
      return;

    case 3:
      exit(0); // exit program with 0 status (fatal subModule failure)
    }
  }
}

void IdleState::leave()
{
  cout << "\nleaving Idle State\n";
}
