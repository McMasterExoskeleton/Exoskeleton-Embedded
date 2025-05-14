#pragma once
#include <vector>

// Input: 2D vector of sensor data with shape [timesteps][6]
// Output: processed in-place
void process_sensor_data(std::vector<std::vector<float>>& data, int section_size = 100, float diff_max = 200.0f);
