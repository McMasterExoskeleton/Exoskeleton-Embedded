/* 
 * Estimates the velocity and acceleration of a joint, using numerical differentation
 *
 * Usage:
 *   JointEstimator estimator(); 
 *   estimator.update(AI_desired_angle, time_difference);
 *   double velocity = estimator.getVelocity();
 *   double acceleration = estimator.getAcceleration();
*/

#include "JointEstimator.h"

JointEstimator::JointEstimator()
    : prev_angle(0.0), prev_velocity(0.0),
      velocity(0.0), acceleration(0.0),
      first_update(true) {}

void JointEstimator::update(double current_angle, double dt) {

    if (dt <= 0.015) return; // skip super fast samples
    
    if (first_update) {
        velocity = 0.0;
        acceleration = 0.0;
        first_update = false;
    } else {
        velocity = (current_angle - prev_angle) / dt;
        acceleration = (velocity - prev_velocity) / dt;
    }

    prev_angle = current_angle;
    prev_velocity = velocity;
}

void JointEstimator::prime(double angle) {
    prev_angle = angle;
    prev_velocity = 0.0;
    velocity = 0.0;
    acceleration = 0.0;
    first_update = false;
}


double JointEstimator::getVelocity() const {
    return velocity;
}

double JointEstimator::getAcceleration() const {
    return acceleration;
}
