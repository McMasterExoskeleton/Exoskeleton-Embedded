#include "one_d_kalman_filter.h"
#include "low_pass_filter.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <array>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>

using json = nlohmann::json;

// Structure to hold IMU data
struct IMUData {
    std::string timestamp;
    std::string location;
    Eigen::VectorXd measurement;  // heading, pitch, roll
};

// Read IMU data from JSON file
std::vector<IMUData> read_imu_data(const std::string &filename) {
    std::vector<IMUData> imu_data;
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return imu_data;
    }

    while (std::getline(file, line)) {
        try {
            json entry = json::parse(line);
            std::string timestamp = entry["timestamp"];

            for (const auto &sensor : entry["sensors"]) {
                IMUData data;
                data.timestamp = timestamp;
                data.location = sensor["location"];

                data.measurement = Eigen::VectorXd(3);
                data.measurement << sensor["euler"]["heading"],
                                    sensor["euler"]["pitch"],
                                    sensor["euler"]["roll"];

                imu_data.push_back(data);
            }
        } catch (const json::exception &e) {
            std::cerr << "JSON parsing error: " << e.what() << std::endl;
            continue;
        }
    }

    return imu_data;
}

// Apply 1D Kalman filter per Euler component and sensor
void apply_kalman_filter(std::vector<IMUData> &imu_data) {
    std::unordered_map<std::string, std::array<OneDKalmanFilter, 3>> filters;

    std::ofstream diff_file("filtered_differences.txt");
    if (!diff_file.is_open()) {
        std::cerr << "Failed to open filtered_differences.txt" << std::endl;
        return;
    }

    for (auto &data : imu_data) {
        auto &filter_array = filters[data.location];

        // If not initialized yet, construct filters with initial values
        if (filter_array[0].update(0) == 0.0 && filter_array[1].update(0) == 0.0 && filter_array[2].update(0) == 0.0) {
            filter_array[0] = OneDKalmanFilter(0.01, 1.0, data.measurement(0));
            filter_array[1] = OneDKalmanFilter(0.01, 1.0, data.measurement(1));
            filter_array[2] = OneDKalmanFilter(0.01, 1.0, data.measurement(2));
        }

        Eigen::VectorXd prev = data.measurement;

        data.measurement(0) = filter_array[0].update(data.measurement(0));
        data.measurement(1) = filter_array[1].update(data.measurement(1));
        data.measurement(2) = filter_array[2].update(data.measurement(2));

        diff_file << "Sensor: " << data.location << "\n";
        diff_file << "Timestamp: " << data.timestamp << "\n";
        diff_file << "Before filtering: H=" << prev(0)
                  << ", P=" << prev(1)
                  << ", R=" << prev(2) << "\n";
        diff_file << "After filtering:  H=" << data.measurement(0)
                  << ", P=" << data.measurement(1)
                  << ", R=" << data.measurement(2) << "\n\n";
    }

    diff_file.close();
    std::cout << "Filtered differences saved to filtered_differences.txt" << std::endl;
}

// Save filtered IMU data while keeping original format
void save_filtered_data(const std::vector<IMUData> &imu_data, const std::string &filename, const std::string &original_filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    std::ifstream original_file(original_filename);
    if (!original_file.is_open()) {
        std::cerr << "Failed to open original file: " << original_filename << std::endl;
        return;
    }

    std::string line;
    size_t index = 0;

    while (std::getline(original_file, line)) {
        json entry = json::parse(line);
        json filtered_entry;
        filtered_entry["timestamp"] = entry["timestamp"];
        filtered_entry["sensors"] = json::array();

        for (size_t i = 0; i < entry["sensors"].size(); ++i) {
            json sensor = entry["sensors"][i];

            if (index < imu_data.size() && imu_data[index].location == sensor["location"]) {
                sensor["euler"]["heading"] = imu_data[index].measurement(0);
                sensor["euler"]["pitch"] = imu_data[index].measurement(1);
                sensor["euler"]["roll"] = imu_data[index].measurement(2);
                index++;
            }

            filtered_entry["sensors"].push_back(sensor);
        }

        file << filtered_entry.dump() << std::endl;
    }

    std::cout << "Filtered data saved to " << filename << std::endl;
}

int main() {
    std::string input_file = "C:/Users/Luis/Documents/Exoskeleton/Sensor-logging-branch/Exoskeleton-Embedded/sensors/treadmill_imu_data/imu_data_treadmill_5min_1.9mph.json";
    std::string output_file = "C:/Users/Luis/Documents/Exoskeleton/Sensor-logging-branch/Exoskeleton-Embedded/sensors/treadmill_imu_data/filtered_imu_data.json";

    std::vector<IMUData> imu_data = read_imu_data(input_file);
    if (imu_data.empty()) {
        std::cerr << "No IMU data found." << std::endl;
        return -1;
    }

    apply_kalman_filter(imu_data);
    save_filtered_data(imu_data, output_file, input_file);

    return 0;
}
