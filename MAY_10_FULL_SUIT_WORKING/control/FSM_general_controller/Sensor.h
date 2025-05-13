#ifndef SENSOR_H
#define SENSOR_H

class Sensor
{
public:
  static double max_sensor_reading;
  static double min_sensor_reading;

  // Set the max and min sensor readings
  static void setMaxSensorReading(double maxReading);

  static void setMinSensorReading(double minReading);

  // Getters
  static double getMaxSensorReading();

  static double getMinSensorReading();
};

#endif