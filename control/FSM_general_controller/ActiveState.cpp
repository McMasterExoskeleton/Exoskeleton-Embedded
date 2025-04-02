#include "ActiveState.h"
#include <iostream>
#include "../Joint_Estimator/JointEstimator.h"
#include "../Torque_Controller/TorqueController.h"
#include "../IMU_DataLoader/IMUDataLoader.h"

#include <iostream>
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

using namespace IMUDataLoader;

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
    TorqueController controller(params);

    // Load IMU Data
    JointData knee_data = loadJointData("joint_data.json", "Right Knee");
    const std::vector<double>& timestamps = knee_data.timestamps;
    const std::vector<double>& knee_angles = knee_data.angles_rad;

    // These variables are only for testing purposes
    // We want to see how the coriolis and gravity term affect the torque output
    double hip_m_acc, hip_c_vel, hip_g;
    double knee_m_acc, knee_c_vel, knee_g;

    JointEstimator hip_estimator;
    JointEstimator knee_estimator;

    hip_estimator.prime((10.0 * M_PI / 180.0) * std::sin(2 * M_PI * 1.0 * timestamps[0]));
    knee_estimator.prime(knee_angles[0]);

    // Setup logging
    std::ofstream log("Testing/log_output.csv");
    log << "time,hip_angle,hip_velocity,hip_acceleration,"
           "hip_m_acc,hip_c_vel,hip_g,hip_torque,"
           "knee_angle,knee_velocity,knee_acceleration,"
           "knee_m_acc,knee_c_vel,knee_g,knee_torque\n";
    /////////////////////////////////////////////////////////////////////////////
    for (size_t i = 1; i < knee_angles.size(); ++i) 
    {
        double t = timestamps[i];
        double dt = timestamps[i] - timestamps[i - 1];
      
        // retreive timer pointer
        Timer *timer = Timer::instance();
        timer->Reset();

        // Run simulation for 10 seconds for testing purposes (TODO: maybe create a non-blocking keyboard interrupt to change states)
        if (t >= 120.0) {
            std::cout << "Finished 120-second test.\n";
            leave();
            current = idle;
            return;
        }
        timer->Tick(); // time measurement

        // Simulated joint angles (TODO: replace with AI later)
        // make sure angle is in RADIANS!!
        double hip_angle = (10.0 * M_PI / 180.0) * std::sin(2 * M_PI * 1.0 * t);
        double knee_angle = knee_angles[i];

        // Update speed, acceleration estimator
        hip_estimator.update(hip_angle, dt);
        knee_estimator.update(knee_angle, dt);

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
        std::cout << "Time: " << t << "\tHip Torque: " << hip_torque << "\tKnee Torque: " << knee_torque << "\n";

        // Log to CSV
        log << t << ","
            << hip.angle << "," << hip.velocity << "," << hip.acceleration << ","
            << hip_m_acc << "," << hip_c_vel << "," << hip_g << "," << hip_torque << ","
            << knee.angle << "," << knee.velocity << "," << knee.acceleration << ","
            << knee_m_acc << "," << knee_c_vel << "," << knee_g << "," << knee_torque << "\n";
        
        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }
}

void ActiveState::leave()
{
    cout << "\nleaving Active State\n";
}
