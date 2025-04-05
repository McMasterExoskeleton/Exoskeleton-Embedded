#ifndef ONE_D_KALMAN_FILTER_H
#define ONE_D_KALMAN_FILTER_H

class OneDKalmanFilter {
public:
    bool initialized;

    OneDKalmanFilter();
    OneDKalmanFilter(double process_noise, double measurement_noise, double initial_estimate = 0.0, double initial_error = 1.0);

    void initialize(double initial_estimate);
    double update(double measurement);

private:
    double x;  // Estimated state
    double p;  // Estimated error covariance
    double q;  // Process noise covariance
    double r;  // Measurement noise covariance
};

#endif // ONE_D_KALMAN_FILTER_H
