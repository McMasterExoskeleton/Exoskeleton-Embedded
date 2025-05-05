#pragma once

#include <vector>


#include "../torque_controller/torqueController.h"

class JointEstimator {
 public:
  JointEstimator();
  TorqueController::JointState update(double current_angle, double dt);

 private:
  double prev_angle;
  double prev_velocity;
  double velocity;
  double acceleration;
};
