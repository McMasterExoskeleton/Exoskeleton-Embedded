#include "clean.h"

#include <vector>

// Constants for knee and hip offsets
constexpr double KNEE_OFFSET = 0;  // Example value
constexpr double HIP_OFFSET = 0;   // Example value

// Function to clean the offset in AI data readings
std::vector<float> cleanAIOffsets(const std::vector<float>& imuReadings) {
  std::vector<float> cleanedReadings(6);

      cleanedReadings[0] = imuReadings[0] - KNEE_OFFSET; // lk
      cleanedReadings[4] = imuReadings[4] - KNEE_OFFSET; // rk

      cleanedReadings[3] = imuReadings[3] - HIP_OFFSET; // rh
      cleanedReadings[5] = imuReadings[5] - HIP_OFFSET; // lh

  // Fixing Knee angles to be relative to the hip
  cleanedReadings[0] = cleanedReadings[0] - cleanedReadings[5]; // left leg
  cleanedReadings[4] = cleanedReadings[4] - cleanedReadings[3]; // right leg

  return cleanedReadings;
}