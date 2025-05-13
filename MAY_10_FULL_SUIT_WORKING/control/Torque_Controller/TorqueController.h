#pragma once

#include <array>

class TorqueController {
public:
    struct JointState {
        double angle;
        double velocity;
        double acceleration;
    };

    /*
     * ModelParameters
     * ----------------
     * Physical parameters of the limb segments
     *
     *   m1  - Mass of thigh
     *   m2  - Mass of shank
     *   L1  - Length of thigh
     *   L2  - Length of shank
     *   r1  - Distance from hip joint to COM of thigh
     *   r2  - Distance from knee joint to COM of shank
     *   I1  - Moment of inertia of thigh about COM
     *   I2  - Moment of inertia of shank about COM
     *   g   - Gravity constant 
     */
    struct ModelParameters {
        double m1, m2;
        double L1, L2;
        double r1, r2;
        double I1, I2;
        double g;
    };

    TorqueController(const ModelParameters& params);
    void computeTorque(const JointState& hip, const JointState& knee, double& hip_torque, double& knee_torque, double& hip_m_acc, double& hip_c_vel, double& hip_g, double& knee_m_acc, double& knee_c_vel, double& knee_g);

private:
    ModelParameters params;
    void computeMassMatrix(double q1, double q2, std::array<std::array<double, 2>, 2>& M);
    void computeCoriolisMatrix(double q1, double q2, double dq1, double dq2, std::array<std::array<double, 2>, 2>& C);
    void computeGravityVector(double q1, double q2, std::array<double, 2>& G);
};

