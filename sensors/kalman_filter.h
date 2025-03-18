#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter() : KalmanFilter(3, 3) {} // Default constructor calls the parameterized one
    KalmanFilter(int state_size, int measurement_size);
    void initialize();
    Eigen::VectorXd filter(const Eigen::VectorXd& measurement);
    
// private:
    Eigen::VectorXd x;  // State vector
    Eigen::MatrixXd P;  // Estimation uncertainty covariance
    Eigen::MatrixXd Q;  // Process noise covariance
    Eigen::MatrixXd R;  // Measurement noise covariance
    Eigen::MatrixXd K;  // Kalman gain
    Eigen::MatrixXd I;  // Identity matrix
};

#endif // KALMAN_FILTER_H
