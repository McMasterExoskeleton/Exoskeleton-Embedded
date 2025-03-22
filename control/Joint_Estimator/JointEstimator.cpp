/* 
 * Estimates the velocity and acceleration of a joint, using numerical differentation
 *
 * Usage:
 *   JointEstimator estimator(0.01); // 10 ms timestep
 *   estimator.update(AI_desired_angle);
 *   double velocity = estimator.getVelocity();
 *   double acceleration = estimator.getAcceleration();
*/

#include "JointEstimator.h"

JointEstimator::JointEstimator(double dt) : dt(dt), prev_angle(0), prev_velocity(0), velocity(0), acceleration(0) {}

void JointEstimator::update(double current_angle) {
    velocity      = (current_angle - prev_angle) / dt;
    acceleration  = (velocity - prev_velocity) / dt;

    prev_angle    = current_angle;
    prev_velocity = velocity;
}

double JointEstimator::getVelocity() const 
{
    return velocity; 
}

double JointEstimator::getAcceleration() const 
{
    return acceleration;
}
