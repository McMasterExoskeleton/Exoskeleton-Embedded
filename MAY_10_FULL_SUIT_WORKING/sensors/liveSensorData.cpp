#include "liveSensorData.h"
#include "data_collection.h"
#include "bno055.h"
#include "rpi_tca9548a.h"

// const int maxBufferSize = 30;



std::vector<double> liveSensorData(std::vector<bno055_t>& sensors, rpi_tca9548a& tca) {
    std::vector<double> roll_values;

    if (!initI2C()) return {};

    std::string filename = "imu_data.json";
    logFile.open(filename, std::ios::app);
    if (!logFile.is_open()) {
        std::cerr << "Failed to open imu_data.json for writing" << std::endl;
        return {};
    }

    json logEntry;
    logEntry["timestamp"] = get_human_readable_timestamp();
    logEntry["sensors"] = json::array();

    for (size_t i = 0; i < sensors.size(); i++) {
        bno055_euler_double_t euler;

        tca.set_channel(sensorChannelsBackup[i]);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (bno055_convert_double_euler_hpr_deg(&euler) != BNO055_SUCCESS) {
            std::cerr << "Failed to read euler from: " << sensorLocations[i] << std::endl;
            roll_values.push_back(0.0);
            continue;
        }

        std::cout << sensorLocations[i] << ": "
                  << "Heading: " << euler.h
                  << ", Roll: " << euler.r
                  << ", Pitch: " << euler.p << std::endl;

        json sensorData;
        sensorData["location"] = sensorLocations[i];
        sensorData["euler"] = {{"heading", euler.h}, {"roll", euler.r}, {"pitch", euler.p}};
        logEntry["sensors"].push_back(sensorData);

        roll_values.push_back(euler.r);
    }

    logFile << logEntry.dump() << std::endl;
    logFile.flush();
    logFile.close();

    return roll_values;
}

