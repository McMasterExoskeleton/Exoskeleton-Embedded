#ifndef LIVE_SENSOR_DATA_H
#define LIVE_SENSOR_DATA_H

#include "bno055.h"
#include "rpi_tca9548a.h"
#include <vector>
#include <cmath>


std::vector<double> liveSensorData(std::vector<bno055_t>& sensors, rpi_tca9548a& tca);

#endif // LIVE_SENSOR_DATA_H
