#include "ActiveState.h"
#include <iostream>
#include "../Joint_Estimator/JointEstimator.h"
#include "../Torque_Controller/TorqueController.h"
#include <cmath>
#include <chrono>
#include <thread>
#include <fstream>
#include "StateParameters.h"
#include "Timer.h"
#define MAX_THRESHOLD_TIME 5.0f
#define MIN_KNEE_ANGLE -15.0
#define MAX_KNEE_ANGLE 130.0
#define MAX_HIP_ANGLE 120
#define MIN_HIP_ANGLE -30

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
    // activate motors
}

void ActiveState::update()
{

    TorqueController::ModelParameters params = StateParameters::initializeModelParams(); // moved to stateparameters file

    double dt = 0.01; // 10 ms loop
    double t = 0.0;

    TorqueController controller(params);
    JointEstimator hip_estimator(dt);
    JointEstimator knee_estimator(dt);

    // Setup logging
    std::ofstream log("Testing/log_output.csv");
    log << "time,hip_angle,hip_velocity,hip_acceleration,"
           "hip_m_acc,hip_c_vel,hip_g,hip_torque,"
           "knee_angle,knee_velocity,knee_acceleration,"
           "knee_m_acc,knee_c_vel,knee_g,knee_torque\n";
    /////////////////////////////////////////////////////////////////////////////

    // retreive timer pointer
    Timer *timer = Timer::instance();
    timer->Reset();

    while (true)
    {

        // Run simulation for 10 seconds for testing purposes (TODO: maybe create a non-blocking keyboard interrupt to change states)
        if (t >= 10.0)
        {
            std::cout << "Finished 10-second test.\n";
            leave();
            current = idle;
            return;
        }

        timer->Tick(); // time measurement

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
            .acceleration = hip_estimator.getAcceleration()};

        TorqueController::JointState knee = {
            .angle = knee_angle,
            .velocity = knee_estimator.getVelocity(),
            .acceleration = knee_estimator.getAcceleration()};

        // Compute torque
        double hip_torque = 0.0, knee_torque = 0.0;
        controller.computeTorque(
            hip, knee,
            hip_torque, knee_torque,
            hip_m_acc, hip_c_vel, hip_g,
            knee_m_acc, knee_c_vel, knee_g);

        // state transitions

        // error state
        if (knee_torque > Sensor::getMaxSensorReading() || knee_angle > MAX_KNEE_ANGLE || knee_angle < MIN_KNEE_ANGLE || hip_angle > MAX_HIP_ANGLE || hip_angle < MIN_HIP_ANGLE)
        {
            leave();
            current = error;
            cout << "\n\nTorque sensor value or angle value is too large. Possible error. Move to error state.\n\n";
            return;
        }

        // idle state

        if (knee_torque < Sensor::getMinSensorReading() && hip_torque < Sensor::getMinSensorReading())
        {
            timer->Tick(); // time measurement

            if (timer->DeltaTime() > MAX_THRESHOLD_TIME)
            {

                leave();
                current = idle;
                cout << "\n\nmax_threshold_time expired. Moving to idle state.\n\n";
                return;
            }
        }
        else
        {
            // last_movement_time = 0 since we are continuing in the active state
            timer->Reset();
        }

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
