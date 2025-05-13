/*
 * Computes the desired assistive torques for the hip and knee joints based on
 * joint angles, velocities, accelerations, and a Mass Matrix
 *
 * The torque is calculated using:
 *   T = M(acceleration) * q̈ + C(velocity) * G(angle)
 *
 * Model derivation and justification are provided in the "Exoskeleton System Controller" document
 */

#include "TorqueController.h"
#include <cmath>
#include <array>

TorqueController::TorqueController(const ModelParameters& p) : params(p) {}

void TorqueController::computeTorque(const JointState& hip, const JointState& knee, double& hip_torque, double& knee_torque, double& hip_m_acc, double& hip_c_vel, double& hip_g, double& knee_m_acc, double& knee_c_vel, double& knee_g) {
    std::array<std::array<double, 2>, 2> M, C;
    std::array<double, 2> G;

    computeMassMatrix(hip.angle, knee.angle, M);
    computeCoriolisMatrix(hip.angle, knee.angle, hip.velocity, knee.velocity, C);
    computeGravityVector(hip.angle, knee.angle, G);

    std::array<double, 2> acc = { hip.acceleration, knee.acceleration };
    std::array<double, 2> vel = { hip.velocity, knee.velocity };

    // Compute each torque component
    hip_m_acc = M[0][0]*acc[0] + M[0][1]*acc[1];
    hip_c_vel = C[0][0]*vel[0] + C[0][1]*vel[1];
    hip_g     = G[0];

    knee_m_acc = M[1][0]*acc[0] + M[1][1]*acc[1];
    knee_c_vel = C[1][0]*vel[0] + C[1][1]*vel[1];
    knee_g     = G[1];

    // Total torque
    hip_torque = hip_m_acc + hip_c_vel;
    knee_torque = knee_m_acc + knee_c_vel;
}

void TorqueController::computeMassMatrix(double q1, double q2, std::array<std::array<double, 2>, 2>& M) {
    const auto& p = params;
    M[0][0] = p.m1 * p.r1 * p.r1 + p.I1 + p.m2 * (p.L1 * p.L1 + p.r2 * p.r2 + 2 * p.L1 * p.r2 * std::cos(q2));
    M[0][1] = p.m2 * (p.L1 * p.r2 * std::cos(q2) + p.r2 * p.r2) + p.I1;
    M[1][0] = M[0][1];
    M[1][1] = p.m2 * p.r2 * p.r2 + p.I2;
}

void TorqueController::computeCoriolisMatrix(double q1, double q2, double dq1, double dq2, std::array<std::array<double, 2>, 2>& C) {
    const auto& p = params;
    double s = std::sin(q2);
    double term = p.m2 * p.L1 * p.r2 * s;

    C[0][0] = -term * dq2;
    C[0][1] = -term * dq1 - term * dq2;
    C[1][0] =  term * dq1;
    C[1][1] = 0;
}

void TorqueController::computeGravityVector(double q1, double q2, std::array<double, 2>& G) {
    const auto& p = params;
    G[0] = p.m1 * p.g * p.r1 * std::cos(q1) + p.m2 * p.g * (p.L1 * std::cos(q1) + p.r2 * std::cos(q1 + q2));
    G[1] = p.m2 * p.g * p.r2 * std::cos(q1 + q2);
}
