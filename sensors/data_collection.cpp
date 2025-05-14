#include "data_collection.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <csignal>
#include <ctime>
#include <string>
#include <wiringPiI2C.h>
#include <unistd.h>
#include "bno055.h"
#include "rpi_tca9548a.h"
#include <nlohmann/json.hpp>


using json = nlohmann::json;
using namespace std::chrono;
using namespace std::this_thread;

std::ofstream logFile;
bool running = true;
rpi_tca9548a tca;
int fd;

const std::vector<int> sensorChannels = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20};
const std::vector<int> sensorChannelsBackup = {0, 1, 2, 3, 4, 5};
const std::vector<std::string> sensorLocations = {"Left Hip", "Left Knee", "Left Ankle", "Right Ankle","Right Knee", "Right Hip"};

// void signal_handler(int signal) {
//     std::cout << "\nStopping data logging..." << std::endl;
//     running = false;
// }

bool initI2C() {
    fd = wiringPiI2CSetup(0x29);
    if (fd == -1) {
        std::cerr << "Failed to initialize I2C connection!" << std::endl;
        return false;
    }
    return true;
}

s8 I2C_bus_write_with_tca(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    if (fd == -1) return BNO055_ERROR;
    for (int i = 0; i < cnt; i++) {
        if (wiringPiI2CWriteReg8(fd, reg_addr + i, reg_data[i]) == -1) {
            return BNO055_ERROR;
        }
    }
    return BNO055_SUCCESS;
}

s8 I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    if (fd == -1) return BNO055_ERROR;
    for (int i = 0; i < cnt; i++) {
        reg_data[i  ] = wiringPiI2CReadReg8(fd, reg_addr + i);
        if (reg_data[i] == -1) return BNO055_ERROR;
    }
    return BNO055_SUCCESS;
}

s8 read_euler_angles(bno055_t* imu, bno055_euler_double_t* euler) {
    if (bno055_convert_double_euler_hpr_deg(euler) != BNO055_SUCCESS) {
        std::cerr << "Failed to read Euler angles!" << std::endl;
        return BNO055_ERROR;
    }
    return BNO055_SUCCESS;
}

// Initialize the IMU sensor (Assign function pointers and set device address)
s8 initialize_imu(bno055_t* imu, u8 dev_addr) {
    imu->bus_write  = I2C_bus_write_with_tca;
    imu->bus_read   = I2C_bus_read;
    imu->delay_msec = delay_msec;
    imu->dev_addr   = dev_addr;

    if (bno055_init(imu) != BNO055_SUCCESS) {
        std::cerr << "BNO055 initialization failed!" << std::endl;
        return BNO055_ERROR;
    }
    return BNO055_SUCCESS;
}

// Configure the IMU operation mode
s8 setup_imu(bno055_t* imu) {
    if (bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF) != BNO055_SUCCESS) {
        std::cerr << "Failed to set operation mode!" << std::endl;
        return BNO055_ERROR;
    }
    delay_msec(BNO055_MODE_SWITCHING_DELAY);
    return BNO055_SUCCESS;
}

// void initialize_sensors_test(std::vector<bno055_t> sensors, rpi_tca9548a tca , int addr){
//     // int addr = 0x29;
//     // std::vector<bno055_t> sensors;// Vector to store sensor objects
//     for (size_t i = 0; i < sensorChannels.size(); i++) {
//         tca.set_channel(sensorChannelsBackup[i]);
//         usleep(10000);
//         bno055_t imu;

//         if (initialize_imu(&imu, addr) != BNO055_SUCCESS) {
//             std::cerr << "Failed to initialize sensor at address 0x" << std::hex << addr << std::endl;
//             continue;  // Skip this sensor and continue with others
//         }
//         if (setup_imu(&imu) != BNO055_SUCCESS) {
//             std::cerr << "Failed to set operation mode for sensor at address 0x" << std::hex << addr << std::endl;
//             continue;
//         }
//         sensors.push_back(imu);
//     }
// }

void delay_msec(u32 msec) {
    usleep(msec * 1000);
}

std::string get_human_readable_timestamp() {
    auto now = system_clock::now();
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    std::time_t now_c = system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&now_c);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &local_tm);
    return std::string(buffer) + "." + std::to_string(ms.count());
}

// int main(int argc, char* argv[]) {
//     signal(SIGINT, signal_handler);
//     if (!initI2C()) return -1;
//     tca.init(0x70);
    
//     std::string filename;
//     if (argc > 1) {
//         filename = argv[1];  // Take filename from command-line argument
//     } else {
//         std::cout << "Enter filename for logging (e.g., imu_data.json): ";
//         std::cin >> filename;
//     }

//     logFile.open(filename, std::ios::app);
//     if (!logFile.is_open()) {
//         std::cerr << "Failed to open imu_data.json for writing" << std::endl;
//         return -1;
//     }
    
//     std::vector<bno055_t> sensors(5);
//     for (size_t i = 0; i < sensorChannels.size(); i++) {
//         tca.set_channel(sensorChannelsBackup[i]);
//         usleep(10000);

//         initialize_imu(sensors[i], 0x29);
//         setup_imu(sensors[i]);
//     }

//     int sampleIntervalMs = 2000;
//     while (running) {
//         auto start_time = steady_clock::now();
        
//         auto t1 = steady_clock::now();
//         json logEntry;
//         logEntry["timestamp"] = get_human_readable_timestamp();
//         logEntry["sensors"] = json::array();
//         auto t2 = steady_clock::now();

//         for (size_t i = 0; i < sensors.size(); i++) {
//             auto sensor_start = steady_clock::now();
            
//             tca.set_channel(sensorChannelsBackup[i]);
//             auto t3 = steady_clock::now();
            
//             bno055_euler_double_t euler;
//             bno055_accel_double_t accel;
//             bno055_linear_accel_double_t linear_accel;
//             bno055_gravity_double_t gravity;
//             bno055_gyro_double_t angular_velocity;
//             bno055_mag_double_t mag_field;
//             bno055_quaternion_t quater;

//             if (bno055_convert_double_euler_hpr_deg(&euler) != BNO055_SUCCESS) {
//                 std::cerr << "Failed to read euler from: " << sensorLocations[i] << std::endl;
//             }
//             std::cout << sensorLocations[i] << ": " << euler.h << " Roll: " << euler.r << " Pitch: " << euler.p << std::endl;
//             if (euler.h == 0.0 && euler.r == 0.0 && euler.p == 0.0){
//                 std::cerr << "BAD READING ALL ZEROS " << std::endl;
//             }
//             //bno055_convert_double_accel_xyz_msq(&accel);
//             //bno055_convert_double_linear_accel_xyz_msq(&linear_accel);
//             //bno055_convert_double_gravity_xyz_msq(&gravity);
//             //bno055_convert_double_gyro_xyz_dps(&angular_velocity);
//             //bno055_convert_double_mag_xyz_uT(&mag_field);
//             //bno055_read_quaternion_wxyz(&quater);

//             auto t4 = steady_clock::now();

//             json sensorData;
//             sensorData["location"] = sensorLocations[i];
//             sensorData["euler"] = {{"heading", euler.h}, {"roll", euler.r}, {"pitch", euler.p}};
//             // sensorData["acceleration"] = {{"x", accel.x}, {"y", accel.y}, {"z", accel.z}};
//             // sensorData["linear_acceleration"] = {{"x", linear_accel.x}, {"y", linear_accel.y}, {"z", linear_accel.z}};
//             // sensorData["gravity"] = {{"x", gravity.x}, {"y", gravity.y}, {"z", gravity.z}};
//             // sensorData["angular_velocity"] = {{"x", angular_velocity.x}, {"y", angular_velocity.y}, {"z", angular_velocity.z}};
//             // sensorData["magnetic_field"] = {{"x", mag_field.x}, {"y", mag_field.y}, {"z", mag_field.z}};
//             //sensorData["quaternion"] = {{"w", quater.w}, {"x", quater.x}, {"y", quater.y}, {"z", quater.z}};
            
//             logEntry["sensors"].push_back(sensorData);
            
//             auto sensor_end = steady_clock::now();

//             // std::cout << "Sensor " << i << " switch time: " 
//             //         << duration_cast<milliseconds>(t3 - sensor_start).count() << " ms" << std::endl;

//             std::cout << "Sensor " << i << " read time: " 
//                     << duration_cast<milliseconds>(t4 - t3).count() << " ms" << std::endl;

//             // std::cout << "Sensor " << i << " JSON prep time: " 
//             //         << duration_cast<milliseconds>(sensor_end - t4).count() << " ms" << std::endl;
//         }

//         auto t5 = steady_clock::now();
//         logFile << logEntry.dump() << std::endl;
//         logFile.flush();
//         auto t6 = steady_clock::now();

//         // std::cout << "File write & flush time: " 
//         //         << duration_cast<milliseconds>(t6 - t5).count() << " ms" << std::endl;

//         auto end_time = steady_clock::now();
//         // std::cout << "Total loop time: " 
//         //         << duration_cast<milliseconds>(end_time - start_time).count() << " ms" << std::endl;

//         milliseconds sleep_time = milliseconds(sampleIntervalMs) - duration_cast<milliseconds>(end_time - start_time);
//         if (sleep_time.count() > 0) sleep_for(sleep_time);
//         std::cout << " ------------- " << std::endl;
//     }

//     logFile.close();

//     return 0;
// }
