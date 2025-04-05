#include "one_d_kalman_filter.h"

// Default constructor
OneDKalmanFilter::OneDKalmanFilter()
    : x(0.0), p(1.0), q(1e-5), r(1e-2), initialized(false) {}

// Constructor with parameters
OneDKalmanFilter::OneDKalmanFilter(double process_noise, double measurement_noise, double initial_estimate, double initial_error)
    : x(initial_estimate), p(initial_error), q(process_noise), r(measurement_noise), initialized(true) {}

// Initialize manually (for lazy init in loop)
void OneDKalmanFilter::initialize(double initial_estimate) {
    x = initial_estimate;
    p = 1.0;
    initialized = true;
}

// Main Kalman update step
double OneDKalmanFilter::update(double measurement) {
    // Prediction
    p += q;

    // Update
    double k = p / (p + r); // Kalman gain
    x = x + k * (measurement - x);
    p = (1 - k) * p;

    return x;
}
