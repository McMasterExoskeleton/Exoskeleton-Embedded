#include <torch/script.h>
#include <torch/torch.h>

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <atomic>
#include "../sensors/bno055.h"
#include "../sensors/rpi_tca9548a.h"

#include "../models/model.h"           // header file for model
#include "../motors/motor_api.h"       // header file for motor API
#include "../sensors/getSensorData.h"  // header file for normzalied sensor data
#include "clean_angles/clean.h"        // header file for clean angles
#include "joint_estimator/jointEstimator.h"
#include "torque_controller/torqueController.h"
#include "../sensors/sensor_preprocessing.h"
#include "../sensors/data_collection.h" 
#include "../sensors/liveSensorData.h"   // header file for sensor preprocessing

// test settings
const bool SIMULATION = false;  // true when running without motors
const int MOTOR_NUMBER = 0;   // number of motors 0-4
const int SLEEP_TIME = 100;   // sleep time in ms
std::vector<std::vector<float>> full_sensor_buffer;

int16_t clampTorque(int16_t torque, int16_t min, int16_t max) {
  return std::max(min, std::min(max, torque));
}

// Function to limit torque change per time step
int16_t limitTorqueChange(int16_t currentTorque, int16_t previousTorque, int16_t maxChange) {
  int16_t delta = currentTorque - previousTorque;
  if (std::abs(delta) > maxChange) {
    return previousTorque + (delta > 0 ? maxChange : -maxChange);
  }
  return currentTorque;
}

// to ensure motors shut down on Ctrl+C
static std::vector<Motor>* g_motors_ptr = nullptr;
volatile sig_atomic_t shutdown_requested = 0;
// void signalHandler(int signum) {
//   std::cout << "\nCaught signal " << signum << ", disabling motors...\n";
//   if (g_motors_ptr) {
//     for (auto& m : *g_motors_ptr) {
//       m.stopMotor();
//       m.disconnectMotor();
//     }
//   }
//   std::exit(signum);
// }
void signalHandler(int signum) {
  shutdown_requested = 1;
  tca.no_channel();
}


std::string get_current_timestamp() {
  auto now = std::time(nullptr);
  char buf[100];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", localtime(&now));
  return std::string(buf);
}

void log_message(const std::string& message) {
  std::cout << get_current_timestamp() << " - " << message << std::endl;
}

int main() {
  std::vector<Motor> motors;  // vector of motors
  double dt = 0.1;            // Sampling rate for model
  int chunk_index = 0;
  std::vector<int16_t> previous_torque_values(4, 0);
  int addr = 0x29;
  // std::vector<bno055_t> sensors;// Vector to store sensor objects

  // Load the model
  // load the models weights into memory
  torch::jit::script::Module model;
  try {
    model = torch::jit::load("../model.pt");
    model.eval();
    log_message("Model loaded successfully.");
    std::cout <<  std::endl;
  } catch (const c10::Error& e) {
    log_message("Error loading the model: " + std::string(e.what()));
    return -1;
  }

  // Initialize motors
  if (!SIMULATION) {
    motors = {
        Motor("/dev/ttyUSB0", 1),
        Motor("/dev/ttyUSB1", 2),
        // Motor("/dev/ttyUSB2", 3),
        // Motor("/dev/ttyUSB3", 4),
    };

    g_motors_ptr = &motors;

    // register Ctrl+C handler
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    for (auto& m : motors) {
      if (!m.initializeMotor()) {
        std::cerr << "Failed to init motor (slave " << m.getStatus() << ")\n";
        return -1;
      }
      m.setOperationMode(PT_MODE);
      m.setMaxTorque(1000);
    }
  }

  // initialize variables
  constexpr size_t WINDOW = 30;
  std::vector<std::vector<float>> sensor_history(6, std::vector<float>(WINDOW));
  TorqueController::JointState hipLeft, kneeLeft, hipRight,
      kneeRight;  // initializing joints
  std::vector<float> predicted_angles(6), clean_angles(6);
  std::vector<int16_t> torque_values(4);
  const std::string sensor_file =
      "../filtered_imu_data_treadmill_5min_1.9mph.json";

  // initialize sensors
  std::vector<bno055_t> sensors(6);
  rpi_tca9548a tca;
  tca.init(0x70);
  tca.no_channel();

  for (size_t i = 0; i < sensorChannels.size(); i++) {
    tca.set_channel(sensorChannelsBackup[i]);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //bno055_t imu;

    if (initialize_imu(&sensors[i], addr) != BNO055_SUCCESS) {
        std::cerr << "Failed to initialize sensor at address 0x" << std::hex << addr << std::endl;
        continue;  // Skip this sensor and continue with others
    }
    if (setup_imu(&sensors[i]) != BNO055_SUCCESS) {
        std::cerr << "Failed to set operation mode for sensor at address 0x" << std::hex << addr << std::endl;
        continue;
    }
    //sensors.push_back(imu);
  }
  //initialize_sensors_test(sensors, tca, 0x29);
  std::vector<double> euler_roll;

  // Control system configuration/initializing
  JointEstimator hipRight_estimator;
  JointEstimator kneeRight_estimator;
  JointEstimator hipLeft_estimator;
  JointEstimator kneeLeft_estimator;


  // Main loop
  while (true) {
    std::cout << "Running main loop: shutdown requested: " << shutdown_requested << std::endl;
    if (shutdown_requested) break;
    auto loop_start = std::chrono::steady_clock::now();
    
    euler_roll = liveSensorData(sensors, tca);
    std::vector<float> euler_roll_float(euler_roll.begin(), euler_roll.end());
    full_sensor_buffer.push_back(euler_roll_float);

    if (full_sensor_buffer.size() > 100) {
        full_sensor_buffer.erase(full_sensor_buffer.begin(), full_sensor_buffer.end() - 100);
    }

    if (full_sensor_buffer.size() < 30) {
        std::cout << "Waiting for 30 samples...\n";
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
        int remaining_time = SLEEP_TIME - static_cast<int>(elapsed.count());
        if (remaining_time > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(remaining_time));
        }
        continue;
    }
    //auto raw = get_sensor_data(sensor_file, chunk_index, WINDOW);
    // if (raw.size() < WINDOW) {
    //   std::cerr << "Only " << raw.size() << " samples available; need "
    //             << WINDOW << "\n";
    //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //   continue;
    // }

    chunk_index += 1;

    // take the last WINDOW entries and transpose into [6][30]
    // for (size_t j = 0; j < 6; ++j) {
    //   for (size_t t = 0; t < WINDOW; ++t) {
    //     sensor_history[j][t] = raw[t][j];
    //   }
    // }

    // lk la ra rh rk lh

    // running AI model
    std::vector<std::vector<float>> processed = full_sensor_buffer;
    process_sensor_data(processed);

    std::vector<std::vector<float>> input_to_model(processed.end() - 30, processed.end());
    predicted_angles = predict_joint_angles(model, input_to_model);

    // Print predicted angles
    // std::cout << "Predicted Angles:" << std::endl;
    // for (size_t i = 0; i < predicted_angles.size(); ++i) {
    //   std::cout << "  Joint " << i << ": " << predicted_angles[i] <<
    //   std::endl;
    // }

    // clean function call
    clean_angles = cleanAIOffsets(predicted_angles);
    std::transform(clean_angles.begin(), clean_angles.end(), clean_angles.begin(),
               [](float angle) { return angle * M_PI / 180.0f; });

    // Assign angles to the corresponding joints and log the updates
    hipRight = hipRight_estimator.update(clean_angles[3], dt);
    std::cout << "Updated HipRight: angle = " << hipRight.angle 
              << ", velocity = " << hipRight.velocity 
              << ", acceleration = " << hipRight.acceleration << std::endl;

    kneeRight = kneeRight_estimator.update(clean_angles[4], dt);
    std::cout << "Updated KneeRight: angle = " << kneeRight.angle 
              << ", velocity = " << kneeRight.velocity 
              << ", acceleration = " << kneeRight.acceleration << std::endl;

    hipLeft = hipLeft_estimator.update(clean_angles[5], dt);
    std::cout << "Updated HipLeft: angle = " << hipLeft.angle 
              << ", velocity = " << hipLeft.velocity 
              << ", acceleration = " << hipLeft.acceleration << std::endl;

    kneeLeft = kneeLeft_estimator.update(clean_angles[0], dt);
    std::cout << "Updated KneeLeft: angle = " << kneeLeft.angle 
              << ", velocity = " << kneeLeft.velocity 
              << ", acceleration = " << kneeLeft.acceleration << std::endl;

    // torque calculator function call
    TorqueController torqueController;
    torque_values =
        torqueController.computeTorque(hipRight, hipLeft, kneeRight, kneeLeft);
      
    for (size_t i = 0; i < torque_values.size(); ++i) {
      torque_values[i] = clampTorque(torque_values[i], -500, 500);  // Clamp to [-500, 500]
      torque_values[i] = limitTorqueChange(torque_values[i], previous_torque_values[i], 50);  // Limit change to 50
    }
    previous_torque_values = torque_values;
    if (!SIMULATION) {
      if (shutdown_requested) break;
      for (size_t i = 0; i < motors.size(); ++i) {
        motors[i].setTargetTorque(torque_values[i]);
        if (shutdown_requested) break;
        log_message("torque for motor " + std::to_string(i) + ": " +
                    std::to_string(torque_values[i]));
      }
    }

    auto loop_end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
    int remaining_time = SLEEP_TIME - static_cast<int>(elapsed.count());
    if (remaining_time > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(remaining_time));
    }
  }
  std::cout << "Disabling motors...\n";
    for (auto& m : motors) {
        m.disconnectMotor();
    }

    std::cout << "Shutdown complete.\n";
    return 0;
  }
