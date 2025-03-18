#include "kalman_filter.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Structure to hold IMU data
struct IMUData {
    std::string timestamp;
    std::string location; // Track sensor location
    Eigen::VectorXd measurement;
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
            std::string timestamp = entry["timestamp"]; // Keep timestamp as string

            for (auto &sensor : entry["sensors"]) {
                IMUData data;
                data.timestamp = timestamp;
                data.location = sensor["location"]; // Store sensor location

                // Store Euler angles in a vector
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

// Apply Kalman filter to IMU data (per sensor location)
void apply_kalman_filter(std::vector<IMUData> &imu_data) {
    std::unordered_map<std::string, KalmanFilter> filters; // One Kalman filter per sensor location

    // Initialize filters for each sensor location
    for (const auto &data : imu_data) {
        if (filters.find(data.location) == filters.end()) {
            filters[data.location] = KalmanFilter(3, 3);
            filters[data.location].initialize();
            filters[data.location].x = data.measurement; // Initialize with first measurement
        }
    }

    // Open a file to log differences before/after filtering
    std::ofstream diff_file("filtered_differences.txt");
    if (!diff_file.is_open()) {
        std::cerr << "Failed to open filtered_differences.txt" << std::endl;
        return;
    }

    // Apply the corresponding Kalman filter to each sensor's data
    for (auto &data : imu_data) {
        KalmanFilter &kf = filters[data.location]; // Get the correct filter

        Eigen::VectorXd prev_measurement = data.measurement; // Store before filtering
        data.measurement = kf.filter(data.measurement); // Apply Kalman filter

        // Log differences
        diff_file << "Sensor: " << data.location << "\n";
        diff_file << "Timestamp: " << data.timestamp << "\n";
        diff_file << "Before filtering: H=" << prev_measurement(0)
                  << ", P=" << prev_measurement(1)
                  << ", R=" << prev_measurement(2) << "\n";
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
        filtered_entry["timestamp"] = entry["timestamp"]; // Keep original timestamp
        filtered_entry["sensors"] = json::array();

        for (size_t i = 0; i < entry["sensors"].size(); ++i) {
            json sensor = entry["sensors"][i];

            // Find corresponding sensor data in imu_data
            if (index < imu_data.size() && imu_data[index].location == sensor["location"]) {
                // Replace Euler angles with the filtered values
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
