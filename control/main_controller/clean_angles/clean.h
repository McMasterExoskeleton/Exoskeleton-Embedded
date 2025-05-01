#ifndef CLEAN_H
#define CLEAN_H

#include <vector>

// Function to clean the offset in AI data readings
std::vector<float> cleanAIOffsets(const std::vector<float>& imuReadings);

#endif  // CLEAN_H
