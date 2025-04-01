class JointEstimator {
    public:
        JointEstimator();
        void update(double current_angle, double dt);
        void prime(double angle);
        
        double getVelocity()     const;
        double getAcceleration() const;

    private:
        double prev_angle;
        double prev_velocity;
        double velocity;
        double acceleration;

        bool first_update; //skip calculation on first sample
};
