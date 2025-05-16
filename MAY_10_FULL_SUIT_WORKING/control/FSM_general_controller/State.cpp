#include "State.h"
#include "Sensor.h"

State::State()
{
}

// Destructor definition
State::~State()
{
}

double State::getMaxSensorReading()
{
  return Sensor::getMaxSensorReading();
}

double State::getMinSensorReading()
{
  return Sensor::getMinSensorReading();
}