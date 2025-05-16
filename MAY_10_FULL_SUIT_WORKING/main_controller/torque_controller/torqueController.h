#pragma once

#include <vector>
#include <array> 
#include <cstdint>

class TorqueController {
 public:
  double GEAR_RATIO_KNEE = 31.0;  // Gear ratio for torque scaling
  double GEAR_RATIO_HIP = 31.0;   // Gear ratio for torque scaling
  double RATED_TORQUE = 0.64;     // Rated torque for scaling
  double ASSIST_RATE = 0.65;       // Assist rate for scaling

  struct JointState {
    double angle;
    double velocity;
    double acceleration;
  };

  std::vector<int16_t> computeTorque(struct JointState hipRight,
                                       struct JointState hipLeft,
                                       struct JointState kneeRight,
                                       struct JointState kneeLeft);

 private:
  void computeMassMatrix(double q1, double q2,
                         std::array<std::array<double, 2>, 2>& M);
  void computeCoriolisMatrix(double q1, double q2, double dq1, double dq2,
                             std::array<std::array<double, 2>, 2>& C);
  void computeGravityVector(double q1, double q2, std::array<double, 2>& G);
};
