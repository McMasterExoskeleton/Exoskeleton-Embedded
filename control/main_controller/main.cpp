#include <torch/script.h>
#include <torch/torch.h>

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>

#include "../models/model.h"           // header file for model
#include "../motors/motor_api.h"       // header file for motor API
#include "../sensors/getSensorData.h"  // header file for normzalied sensor data
#include "clean_angles/clean.h"        // header file for clean angles
#include "joint_estimator/jointEstimator.h"
#include "torque_controller/torqueController.h"

// test settings
const bool SIMULATION = true;  // true when running without motors
const int MOTOR_NUMBER = 0;   // number of motors 0-4
const int SLEEP_TIME = 100;   // sleep time in ms

// to ensure motors shut down on Ctrl+C
static std::vector<Motor>* g_motors_ptr = nullptr;
void signalHandler(int signum) {
  std::cout << "\nCaught signal " << signum << ", disabling motors...\n";
  if (g_motors_ptr) {
    for (auto& m : *g_motors_ptr) {
      m.stopMotor();
      m.disconnectMotor();
    }
  }
  std::exit(signum);
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
        // Motor("/dev/ttyUSB1", 2),
        // Motor("/dev/ttyUSB2", 3),
        // Motor("/dev/ttyUSB3", 4),
    };

    g_motors_ptr = &motors;

    // register Ctrl+C handler
    std::signal(SIGINT, signalHandler);

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

  // Control system configuration/initializing
  JointEstimator hipRight_estimator;
  JointEstimator kneeRight_estimator;
  JointEstimator hipLeft_estimator;
  JointEstimator kneeLeft_estimator;

  // Main loop
  while (true) {
    auto raw = get_sensor_data(sensor_file, chunk_index, WINDOW);
    if (raw.size() < WINDOW) {
      std::cerr << "Only " << raw.size() << " samples available; need "
                << WINDOW << "\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    chunk_index += 1;

    // take the last WINDOW entries and transpose into [6][30]
    // for (size_t j = 0; j < 6; ++j) {
    //   for (size_t t = 0; t < WINDOW; ++t) {
    //     sensor_history[j][t] = raw[t][j];
    //   }
    // }

    // lk la ra rh rk lh

    // running AI model
    predicted_angles = predict_joint_angles(model, raw);

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

    if (!SIMULATION) {
      for (size_t i = 0; i < motors.size(); ++i) {
        motors[i].setTargetTorque(torque_values[i]);
        log_message("torque for motor " + std::to_string(i) + ": " +
                    std::to_string(torque_values[i]));
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
  }
}
