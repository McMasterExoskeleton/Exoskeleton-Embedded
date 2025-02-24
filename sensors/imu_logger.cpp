#include "driver.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <csignal>
#include <ctime>
#include <string>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std::chrono;
using namespace std::this_thread;

std::ofstream logFile;  // Declare log file globally
bool running = true;    // Flag to control the infinite loop

/**
 * @brief Signal handler to safely stop data logging on SIGINT (Ctrl+C).
 * @param signal The signal number (not used in this case).
 */
void signal_handler(int signal) {
    std::cout << "\nStopping data logging..." << std::endl;
    running = false;
}

/**
 * @brief Generates a human-readable timestamp with milliseconds.
 * @return A string in the format "YYYY-MM-DD HH:MM:SS.mmm".
 */
std::string get_human_readable_timestamp() {
    auto now = system_clock::now();  // Get current time
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;  // Extract milliseconds

    std::time_t now_c = system_clock::to_time_t(now);  // Convert to time_t (seconds)
    std::tm local_tm = *std::localtime(&now_c);  // Convert to local time

    // Buffer for formatted timestamp (YYYY-MM-DD HH:MM:SS)
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &local_tm);

    // Append milliseconds manually
    return std::string(buffer) + "." + std::to_string(ms.count());
}

int main() {
    signal(SIGINT, signal_handler);  // Register signal handler for safe shutdown

    int sampleIntervalMs = 50;  // Sampling rate in milliseconds
    std::vector<int> sensorAddresses = { 0x29 };  // List of I2C sensor addresses

    std::vector<bno055_t> sensors;  // Vector to store sensor objects
    for (int addr : sensorAddresses) {
        bno055_t imu;
        if (initialize_imu(&imu, addr) != BNO055_SUCCESS) {
            std::cerr << "Failed to initialize sensor at address 0x" << std::hex << addr << std::endl;
            continue;  // Skip this sensor and continue with others
        }
        if (setup_imu(&imu) != BNO055_SUCCESS) {
            std::cerr << "Failed to set operation mode for sensor at address 0x" << std::hex << addr << std::endl;
            continue;
        }
        sensors.push_back(imu);
    }

    // Open the log file for writing
    logFile.open("imu_data.json");
    if (!logFile.is_open()) {
        std::cerr << "Failed to open imu_data.json for writing" << std::endl;
        return -1;  // Exit if file cannot be opened
    }

    while (running) {
        auto start_time = steady_clock::now();  // Start time for loop execution measurement

        json logEntry;
        logEntry["timestamp"] = get_human_readable_timestamp();  // Add human-readable timestamp
        logEntry["sensors"] = json::array();  // Initialize JSON array for sensor data

        // Loop through all sensors and record data
        for (auto &imu : sensors) {
            bno055_euler_double_t euler_angles;
            bno055_accel_double_t accel_xyz;
            bno055_linear_accel_double_t linear_accel_xyz;
            bno055_gravity_double_t gravity_xyz;
            bno055_gyro_double_t angular_velocity_xyz;
            bno055_mag_double_t mag_field_xyz;
            bno055_quaternion_t quater_wxyz;

            // Read sensor data
            read_euler_angles(&imu, &euler_angles);
            read_acceleration(&imu, &accel_xyz);
            read_linear_acceleration(&imu, &linear_accel_xyz);
            read_gravity(&imu, &gravity_xyz);
            read_gyro(&imu, &angular_velocity_xyz);
            read_magnetic_field(&imu, &mag_field_xyz);
            read_quaternion(&imu, &quater_wxyz);

            // Store sensor data in JSON format
            json sensorData;
            sensorData["id"] = imu.dev_addr;  // Use the device address as an identifier
            sensorData["euler"] = {
                {"heading", euler_angles.h},
                {"roll", euler_angles.r},
                {"pitch", euler_angles.p}
            };
            sensorData["acceleration"] = {
                {"x", accel_xyz.x},
                {"y", accel_xyz.y},
                {"z", accel_xyz.z}
            };
            sensorData["linear_acceleration"] = {
                {"x", linear_accel_xyz.x},
                {"y", linear_accel_xyz.y},
                {"z", linear_accel_xyz.z}
            };
            sensorData["gravity"] = {
                {"x", gravity_xyz.x},
                {"y", gravity_xyz.y},
                {"z", gravity_xyz.z}
            };
            sensorData["angular_velocity"] = {
                {"x", angular_velocity_xyz.x},
                {"y", angular_velocity_xyz.y},
                {"z", angular_velocity_xyz.z}
            };
            sensorData["magnetic_field"] = {
                {"x", mag_field_xyz.x},
                {"y", mag_field_xyz.y},
                {"z", mag_field_xyz.z}
            };
            sensorData["quaternion"] = {
                {"w", quater_wxyz.w},
                {"x", quater_wxyz.x},
                {"y", quater_wxyz.y},
                {"z", quater_wxyz.z}
            };

            logEntry["sensors"].push_back(sensorData);
        }

        // Write sensor data to the log file
        logFile << logEntry.dump() << std::endl;
        logFile.flush();  // Ensure data is written immediately

        // Measure elapsed time and adjust sleep time accordingly
        auto end_time = steady_clock::now();
        duration<double, std::milli> elapsed_time = end_time - start_time;
        milliseconds sleep_time = milliseconds(sampleIntervalMs) - milliseconds(static_cast<int>(elapsed_time.count()));

        // Ensure we don't sleep for a negative duration
        if (sleep_time.count() > 0) {
            sleep_for(sleep_time);
        }
    }

    logFile.close();  // Close the log file when exiting
    std::cout << "Log file closed. Data logging stopped.\n";
    return 0;
}
