#include "kalman_filter.h"

KalmanFilter::KalmanFilter(int state_size, int measurement_size) {
    x = Eigen::VectorXd::Zero(state_size);   // State initialized to zero
    P = Eigen::MatrixXd::Identity(state_size, state_size); // Identity matrix
    Q = Eigen::MatrixXd::Identity(state_size, state_size) * 0.01;
    R = Eigen::MatrixXd::Identity(measurement_size, measurement_size);
    K = Eigen::MatrixXd::Zero(state_size, measurement_size); 
    I = Eigen::MatrixXd::Identity(state_size, state_size);
}

void KalmanFilter::initialize() {
    x.setZero();  // Reset state to zero
    P.setIdentity();  // Reset covariance to identity
}

Eigen::VectorXd KalmanFilter::filter(const Eigen::VectorXd& measurement) {
    // **Predict step**
    P = P + Q;

    // **Compute Kalman Gain**
    K = P * (P + R).inverse();

    // **Update step**
    x = x + K * (measurement - x);
    P = (I - K) * P;

    return x;
}
