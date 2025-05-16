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

// Apply low-pass filter per location and Euler component
void apply_low_pass_filter(std::vector<IMUData> &imu_data, double alpha = 0.1) {
    std::unordered_map<std::string, std::array<LowPassFilter, 3>> filters;

    std::ofstream log_file("filtered_differences_lpf.txt");
    if (!log_file.is_open()) {
        std::cerr << "Failed to open filtered_differences_lpf.txt" << std::endl;
        return;
    }

    for (auto &data : imu_data) {
        auto &filter_array = filters[data.location];

        for (int i = 0; i < 3; ++i) {
            if (!filter_array[i].is_initialized()) {
                filter_array[i] = LowPassFilter(alpha);
                filter_array[i].initialize(data.measurement(i));
            }

            double before = data.measurement(i);
            double after = filter_array[i].update(data.measurement(i));
            data.measurement(i) = after;

            log_file << "Sensor: " << data.location << ", Timestamp: " << data.timestamp << "\n";
            log_file << "Component " << i << " - Before: " << before << ", After: " << after << "\n";
        }

        log_file << "\n";
    }

    log_file.close();
    std::cout << "LPF differences saved to filtered_differences_lpf.txt\n";
}

// Save filtered IMU data while keeping original format
void save_filtered_lpf(const std::vector<IMUData> &imu_data, const std::string &filename, const std::string &original_filename) {
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

    std::cout << "LPF-filtered data saved to " << filename << std::endl;
}

// Example usage in main
int main() {
    std::string input_file = "sensors/treadmill_imu_data/imu_data_treadmill_5min_1.9mph.json";
    std::string output_file_lpf = "sensors/treadmill_imu_data/filtered_imu_data_lpf.json";

    std::vector<IMUData> imu_data = read_imu_data(input_file);
    if (imu_data.empty()) {
        std::cerr << "No IMU data found." << std::endl;
        return -1;
    }

    apply_low_pass_filter(imu_data, 0.1);  // Use smoothing factor alpha = 0.1
    save_filtered_lpf(imu_data, output_file_lpf, input_file);

    return 0;
}
