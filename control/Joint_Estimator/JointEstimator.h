class JointEstimator {
    public:
        JointEstimator(double dt);
        void update(double current_angle);

        double getVelocity()     const;
        double getAcceleration() const;

    private:
        double dt;
        double prev_angle;
        double prev_velocity;
        double velocity;
        double acceleration;
};
