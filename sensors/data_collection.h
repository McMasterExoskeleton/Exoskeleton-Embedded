#ifndef DATA_COLLECTION_H
#define DATA_COLLECTION_H

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
#include <nlohmann/json.hpp>
#include "bno055.h"
#include "rpi_tca9548a.h"

using json = nlohmann::json;
using namespace std::chrono;
using namespace std::this_thread;

// Globals
extern std::ofstream logFile;
extern bool running;
extern rpi_tca9548a tca;
extern int fd;

extern const std::vector<int> sensorChannels;
extern const std::vector<int> sensorChannelsBackup;
extern const std::vector<std::string> sensorLocations;

// Functions
bool initI2C();
s8 I2C_bus_write_with_tca(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 read_euler_angles(bno055_t* imu, bno055_euler_double_t* euler);
s8 initialize_imu(bno055_t* imu, u8 dev_addr);
s8 setup_imu(bno055_t* imu);
//void initialize_sensors_test(std::vector<bno055_t>& sensors, rpi_tca9548a& tca, int addr);
void delay_msec(u32 msec);
std::string get_human_readable_timestamp();

#endif // DATA_COLLECTION_H
