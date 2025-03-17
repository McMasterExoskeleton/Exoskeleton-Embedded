
#include "Sensor.h"

double Sensor::max_sensor_reading = 0.0; // Initial value for max sensor reading
double Sensor::min_sensor_reading = 0.0; // Initial value for min sensor reading

void Sensor::setMaxSensorReading(double maxReading)
{
  max_sensor_reading = maxReading;
}

void Sensor::setMinSensorReading(double minReading)
{
  min_sensor_reading = minReading;
}

// Getters
double Sensor::getMaxSensorReading()
{
  return max_sensor_reading;
}

double Sensor::getMinSensorReading()
{
  return min_sensor_reading;
}