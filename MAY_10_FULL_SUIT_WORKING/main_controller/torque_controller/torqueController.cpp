/*
 * Computes the desired assistive torques for the hip and knee joints based on
 * joint angles, velocities, accelerations, and a Mass Matrix
 *
 * The torque is calculated using:
 *   T = M(acceleration) * q̈ + C(velocity) * G(angle)
 *
 * Model derivation and justification are provided in the "Exoskeleton System
 * Controller" document
 */

#include "torqueController.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

// Torque Controller Values
const double m1 = 2.3;   // Mass of thigh
const double m2 = 2.3;   // Mass of shank
const double L1 = 0.5;   // Length of thigh
const double L2 = 0.5;   // Length of shank
const double r1 = 0.25;  // Distance from hip joint to COM of thigh
const double r2 = 0.25;  // Distance from knee joint to COM of shank
const double I1 = 0.01;  // Moment of inertia of thigh about COM
const double I2 = 0.01;  // Moment of inertia of shank about COM
const double g = 9.81;   // Gravity constant

std::vector<int16_t> TorqueController::computeTorque(JointState hipRight,
                                                     JointState hipLeft,
                                                     JointState kneeRight,
                                                     JointState kneeLeft) {
  std::vector<int16_t> torque_values(4);

  // Helper lambda to compute torque for a single leg
  auto computeLegTorque = [&](const JointState& hip,
                              const JointState& knee) -> std::vector<int16_t> {
    std::array<std::array<double, 2>, 2> M, C;
    std::array<double, 2> G;

    

    // Compute the Mass, Coriolis, and Gravity components
    computeMassMatrix(hip.angle, knee.angle, M);
    computeCoriolisMatrix(hip.angle, knee.angle, hip.velocity, knee.velocity,
                          C);
    
    std::cout << "hip angle: " << hip.angle << std::endl;
    computeGravityVector(hip.angle, knee.angle, G);

    // Joint accelerations
    std::array<double, 2> acc = {hip.acceleration, knee.acceleration};
    std::array<double, 2> vel = {hip.velocity, knee.velocity};

    // Compute torques
    double hipTorque = (M[0][0] * acc[0]) + (M[0][1] * acc[1]) + (C[0][0] * vel[0]) +
                       (C[0][1] * vel[1]) + G[0];
    
    // debug output for each torque component for hip 
    std::cout << "---- Hip Torque Debug ----" << std::endl;
    std::cout << "Mass Matrix (M):" << std::endl;
    std::cout << "  M[0][0]: " << M[0][0] << ", M[0][1]: " << M[0][1] << std::endl;
    std::cout << "  M[1][0]: " << M[1][0] << ", M[1][1]: " << M[1][1] << std::endl;
    std::cout << "Coriolis Matrix (C):" << std::endl;
    std::cout << "  C[0][0]: " << C[0][0] << ", C[0][1]: " << C[0][1] << std::endl;
    std::cout << "  C[1][0]: " << C[1][0] << ", C[1][1]: " << C[1][1] << std::endl;
    std::cout << "Gravity Vector (G):" << std::endl;
    std::cout << "  G[0]: " << G[0] << ", G[1]: " << G[1] << std::endl;
    std::cout << "Joint Accelerations (acc):" << std::endl;
    std::cout << "  acc[0]: " << acc[0] << ", acc[1]: " << acc[1] << std::endl;
    std::cout << "Joint Velocities (vel):" << std::endl;
    std::cout << "  vel[0]: " << vel[0] << ", vel[1]: " << vel[1] << std::endl;
    
    double kneeTorque = M[1][0] * acc[0] + M[1][1] * acc[1] + C[1][0] * vel[0] +
                        C[1][1] * vel[1] + G[1];
    
    // Scale torques to correct units
    int16_t scaledHipTorque = static_cast<int16_t>(
        (hipTorque * ASSIST_RATE * 1000) / (GEAR_RATIO_HIP * RATED_TORQUE));
    int16_t scaledKneeTorque = static_cast<int16_t>(
        (kneeTorque * ASSIST_RATE * 1000) / (GEAR_RATIO_KNEE * RATED_TORQUE));

    // Debug output for computed torques
    std::cout << "  Hip Torque: " << hipTorque << " (scaled: " << scaledHipTorque
              << "), Knee Torque: " << kneeTorque
              << " (scaled: " << scaledKneeTorque << ")" << std::endl;

    return {scaledHipTorque, scaledKneeTorque};
  };

  // Compute torques for each leg
  std::cout << "  Left Leg Torques: " << std::endl;
  auto leftTorques = computeLegTorque(hipLeft, kneeLeft);
  std::cout << "  Right Leg Torques: " << std::endl;
  auto rightTorques = computeLegTorque(hipRight, kneeRight);

  // Assign computed torques to the output array
  torque_values[0] = rightTorques[0];  // Right hip
  torque_values[1] = -rightTorques[1];  // Right knee
  torque_values[2] = -leftTorques[0];   // Left hip
  torque_values[3] = leftTorques[1];   // Left knee

  // Debug output for final torque values
  std::cout << "  Final Scaled Torque Values: [" << torque_values[0] << ", "
            << torque_values[1] << ", " << torque_values[2] << ", "
            << torque_values[3] << "]" << std::endl << std::endl;

  return torque_values;
}
void TorqueController::computeMassMatrix(
    double q1, double q2, std::array<std::array<double, 2>, 2>& M) {
  M[0][0] =
      m1 * r1 * r1 + I1 + m2 * (L1 * L1 + r2 * r2 + 2 * L1 * r2 * std::cos(q2));
  M[0][1] = m2 * (L1 * r2 * std::cos(q2) + r2 * r2) + I1;
  M[1][0] = M[0][1];
  M[1][1] = m2 * r2 * r2 + I2;
}

void TorqueController::computeCoriolisMatrix(
    double q1, double q2, double dq1, double dq2,
    std::array<std::array<double, 2>, 2>& C) {
  double s = std::sin(q2);
  double term = m2 * L1 * r2 * s;

  C[0][0] = -term * dq2;
  C[0][1] = -term * dq1 - term * dq2;
  C[1][0] = term * dq1;
  C[1][1] = 0;
}

void TorqueController::computeGravityVector(double q1, double q2,
                                            std::array<double, 2>& G) {

  G[0] = -m1 * g * r1 * std::sin(q1) -
         m2 * g * (L1 * std::sin(q1) + r2 * std::sin(q1 + q2));
  G[1] = -m2 * g * r2 * std::sin(q1 + q2);
} 
