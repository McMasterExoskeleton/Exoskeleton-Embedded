#include "ActiveState.h"
#include <iostream>
#include "../Joint_Estimator/JointEstimator.h"
#include "../Torque_Controller/TorqueController.h"
#include "../IMU_DataLoader/IMUDataLoader.h"
#include "../../motors/motor_api.h"

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
const double RATED_TORQUE = 0.64; // Nm
const double SIMULATION_TIME = 9.99;
const double GEAR_RATIO = 30.0;  // CHANGE: What is gear ratio?
const bool SIMULATION = false;

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

void ActiveState::leave()
{
    cout << "\nleaving Active State\n";
}

void ActiveState::update()
{
    TorqueController::ModelParameters params = StateParameters::initializeModelParams(); // moved to stateparameters file
    TorqueController controller(params);

    // Load IMU Data
    //JointData knee_data = loadJointData("joint_data.json", "Right Knee");
    //const std::vector<double>& timestamps = knee_data.timestamps;
    //const std::vector<double>& knee_angles = knee_data.angles_rad;

    // Simulated knee joint angle - ranges from 0 to 45 degrees w/ frequency=1
    const double dt = 0.1;
    const double freq = 1.0;  
    const double amplitude_deg = 45.0;
    std::vector<double> timestamps, knee_angles;

    for (double t = 0.0; t <= 10.0; t += dt) {
        timestamps.push_back(t);
        double angle_deg = amplitude_deg * (std::sin(2 * M_PI * freq * t - M_PI / 2) + 1.0) / 2.0;
        knee_angles.push_back(angle_deg * M_PI / 180.0); // convert to radians
    }
    
    // Testing paramters to verify how acceleartion, coriolis & gravity terms affect the torque output
    double hip_m_acc, hip_c_vel, hip_g;
    double knee_m_acc, knee_c_vel, knee_g;

    JointEstimator hip_estimator;
    JointEstimator knee_estimator;

    knee_estimator.prime(knee_angles[0]);
    hip_estimator.prime(0);

    // Setup logging
    std::ofstream log("Testing/log_output.csv");
    log << "time,hip_angle,hip_velocity,hip_acceleration,"
           "hip_m_acc,hip_c_vel,hip_g,hip_torque,"
           "knee_angle,knee_velocity,knee_acceleration,"
           "knee_m_acc,knee_c_vel,knee_g,knee_torque,"
           "knee_scaled_torque\n";

    Motor motor("/dev/ttyUSB0", 1);  // CHANGE: Adjust device path and ID
    // Initialize Motor
    if (!SIMULATION)
    {

        if (!motor.initializeMotor()) {
            std::cerr << "Failed to initialize motor. Aborting...\n";
            leave();
            current = error;
            return;
        }

        motor.setMaxTorque(1000);
        motor.setOperationMode(4);  // Torque Profile Mode
    }

    /////////////////////////////////////////////////////////////////////////////
    for (size_t i = 1; i < knee_angles.size(); ++i) 
    {
        double t = timestamps[i];
        double dt = timestamps[i] - timestamps[i - 1];
      
        // retreive timer pointer
        Timer *timer = Timer::instance();
        timer->Reset();

        // Run simulation for 10 seconds for testing purposes 
        if (t >= SIMULATION_TIME) {
            if (!SIMULATION)
            {
                motor.stopMotor();
                motor.disconnectMotor();
            }
            std::cout << "Finished " << SIMULATION_TIME << "-second test.\n";
            leave();
            current = idle;
            return;
        }
        timer->Tick(); // time measurement

        // assumption is thigh is held vertical and fixed
        double hip_angle = 0;
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

        // STATE TRANSITIONS

        // error state
        if (knee_torque > Sensor::getMaxSensorReading() || knee_angle > MAX_KNEE_ANGLE || knee_angle < MIN_KNEE_ANGLE || hip_angle > MAX_HIP_ANGLE || hip_angle < MIN_HIP_ANGLE)
        {
            if (!SIMULATION)
            {
                motor.stopMotor();
                motor.disconnectMotor();
            }
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

        // Scale torque to correct units
        double scaled_knee_torque = knee_torque / GEAR_RATIO;
        double scaled_hip_torque = hip_torque / GEAR_RATIO;
        int16_t torque_permille = static_cast<int16_t>((scaled_knee_torque *0.85 / RATED_TORQUE) * 1000.0);

        // Send command to motor
        if (!SIMULATION)
            motor.setTargetTorque(torque_permille);

        // Output torque values to the console
        std::cout << "Time: " << t << "\tScaled Hip Torque: " << scaled_hip_torque << "\tScaled Knee Torque: " << scaled_knee_torque << "\n";

        // Log to CSV
        log << t << ","
            << hip.angle << "," << hip.velocity << "," << hip.acceleration << ","
            << hip_m_acc << "," << hip_c_vel << "," << hip_g << "," << hip_torque << ","
            << knee.angle << "," << knee.velocity << "," << knee.acceleration << ","
            << knee_m_acc << "," << knee_c_vel << "," << knee_g << "," << knee_torque << ","
            << scaled_knee_torque << "\n";
        
        // std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}
