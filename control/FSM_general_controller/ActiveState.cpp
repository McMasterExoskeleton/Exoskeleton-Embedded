#include "ActiveState.h"
#include <iostream>
#include "../Joint_Estimator/JointEstimator.h"
#include "../Torque_Controller/TorqueController.h"

#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <fstream>

// Constructor
ActiveState::ActiveState()
{
}

// Destructor
ActiveState::~ActiveState()
{
}

void ActiveState::enter()
{
  cout << "\nNow in Active State \n";
}

void ActiveState::update()
{

    // Setup parameters (TODO: move these parameter definitions to another file)
    TorqueController::ModelParameters params = {
        .m1 = 5.65, .m2 = 3.48,
        .L1 = 0.41, .L2 = 0.4879,
        .r1 = 0.17, .r2 = 0.1892,
        .I1 = 0.0648, .I2 = 0.0107,
        .g = 9.81
    };

    double dt = 0.01;  // 10 ms loop
    double t = 0.0;

    TorqueController controller(params);
    JointEstimator hip_estimator(dt);
    JointEstimator knee_estimator(dt);

    //Setup logging
    std::ofstream log("Testing/log_output.csv");
    log << "time,hip_angle,hip_velocity,hip_acceleration,"
       "hip_m_acc,hip_c_vel,hip_g,hip_torque,"
       "knee_angle,knee_velocity,knee_acceleration,"
       "knee_m_acc,knee_c_vel,knee_g,knee_torque\n";
    /////////////////////////////////////////////////////////////////////////////

    while (true)
     {

        // Run simulation for 10 seconds for testing purposes (TODO: maybe create a non-blocking keyboard interrupt to change states)
        if (t >= 10.0) {
            std::cout << "Finished 10-second test.\n";
            leave();
            current = idle;
            return;
        }

        // Simulated joint angles (TODO: replace with AI later)
        // make sure angle is in RADIANS!!
        double hip_angle = (10.0 * M_PI / 180.0) * std::sin(2 * M_PI * 1.0 * t);
        double knee_angle = (30.0 * M_PI / 180.0) * std::sin(2 * M_PI * 1.0 * t + M_PI / 4);

        // These variables are only for testing purposes
        // We want to see how the coriolis and gravity term affect the torque output
        double hip_m_acc, hip_c_vel, hip_g;
        double knee_m_acc, knee_c_vel, knee_g;

        // Update speed, acceleration estimator
        hip_estimator.update(hip_angle);
        knee_estimator.update(knee_angle);

        TorqueController::JointState hip = {
            .angle = hip_angle,
            .velocity = hip_estimator.getVelocity(),
            .acceleration = hip_estimator.getAcceleration()
        };

        TorqueController::JointState knee = {
            .angle = knee_angle,
            .velocity = knee_estimator.getVelocity(),
            .acceleration = knee_estimator.getAcceleration()
        };

        // Compute torque
        double hip_torque = 0.0, knee_torque = 0.0;
        controller.computeTorque(
            hip, knee,
            hip_torque, knee_torque,
            hip_m_acc, hip_c_vel, hip_g,
            knee_m_acc, knee_c_vel, knee_g
        );

        // Output to console for now (TODO: replace with motor command)
        std::cout << "Hip Torque: " << hip_torque << "\tKnee Torque: " << knee_torque << "\n";

        // Log to CSV
        log << t << ","
            << hip.angle << "," << hip.velocity << "," << hip.acceleration << ","
            << hip_m_acc << "," << hip_c_vel << "," << hip_g << "," << hip_torque << ","
            << knee.angle << "," << knee.velocity << "," << knee.acceleration << ","
            << knee_m_acc << "," << knee_c_vel << "," << knee_g << "," << knee_torque << "\n";
        
        t += dt;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void ActiveState::leave()
{
  cout << "\nleaving Active State\n";
}
