#include "IMUDataLoader.h"
#include <fstream>
#include <vector>
#include "json.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <ctime>

using json = nlohmann::json;

namespace IMUDataLoader {

    // Helper: Convert timestamp string to time in seconds since epoch
    double parseTimestampToSeconds(const std::string& timestamp_str) {
        std::tm t = {};
        double milliseconds = 0.0;

        std::istringstream ss(timestamp_str);
        ss >> std::get_time(&t, "%Y-%m-%d %H:%M:%S");  // Parse main time portion

        // Now get the fractional seconds (milliseconds)
        if (ss.peek() == '.') {
            ss.ignore();
            std::string ms_str;
            std::getline(ss, ms_str);
            milliseconds = std::stod("0." + ms_str);
        }

        std::time_t time_epoch = std::mktime(&t);  // Convert to epoch time (seconds)
        return static_cast<double>(time_epoch) + milliseconds;
    }

    JointData loadJointData(const std::string& filepath, const std::string& location) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Unable to open file: " + filepath);
        }

        json j;
        file >> j;

        std::vector<double> raw_angles_deg;
        std::vector<double> raw_timestamps;

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;

            json frame = json::parse(line);

            std::string time_str = frame["timestamp"];
            double time_sec = parseTimestampToSeconds(time_str);

            for (const auto& sensor : frame["sensors"]) {
                if (sensor["location"] == location) {
                    double roll = sensor["euler"]["roll"];
                    raw_timestamps.push_back(time_sec);
                    raw_angles_deg.push_back(roll);
                }
            }
        }

        if (raw_angles_deg.empty()) {
            throw std::runtime_error("No data found for location: " + location);
        }

        // Normalize timestamps to start at 0
        double t0 = raw_timestamps.front();
        for (double& t : raw_timestamps) {
            t -= t0;
        }

        // Estimate full extension
        double min_roll = *std::min_element(raw_angles_deg.begin(), raw_angles_deg.end());

        std::vector<double> angles_rad;
        for (double deg : raw_angles_deg) {
            double shifted = deg - min_roll;
            double rad = shifted * M_PI / 180.0;
            angles_rad.push_back(rad);
        }

        return JointData{ raw_timestamps, angles_rad };
    }

}

