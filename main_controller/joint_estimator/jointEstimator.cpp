#include "jointEstimator.h"

#include <iostream>
#include <vector>

#include "../torque_controller/torqueController.h"

// constructor with initialized values
JointEstimator::JointEstimator()
  : prev_angle(0.0), prev_velocity(0.0), velocity(0.0), acceleration(0.0) {}

  TorqueController::JointState JointEstimator::update(double current_angle, double dt) {
    TorqueController::JointState angle_values;
  
    // Optional: Apply a low-pass filter to smooth the angle
    angle_values.angle = current_angle;
  
    // Calculate velocity
    angle_values.velocity = (current_angle - prev_angle) / dt;
    velocity = angle_values.velocity;
  
    // Calculate acceleration
    angle_values.acceleration = (velocity - prev_velocity) / dt;
  
    // Debugging output
    std::cout << "current_angle: " << angle_values.angle
              << ", velocity: " << angle_values.velocity
              << ", acceleration: " << angle_values.acceleration
              << std::endl;
  
    // Update previous values
    prev_angle = current_angle;
    prev_velocity = velocity;
  
    return angle_values;
  }
