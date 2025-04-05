#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <functional>
#include <algorithm>
#include <numeric>
#include <cstdlib>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

std::vector<double> normalize(const std::vector<double>& data, int section_size = 100) {
    int n = data.size();
    int sec = section_size;
    if (n < sec) {
        sec = n;
    }
    double total = 0.0;
    for (int i = 0; i < sec; i++) {
        total += data[i];
    }
    double translation = -total / sec;
    std::vector<double> new_data;
    new_data.reserve(n);
    
    for (int i = 0; i < sec; i++) {
        new_data.push_back(data[i] + translation);
    }
    for (int i = sec; i < n; i++) {
        total = total + data[i] - data[i - sec];
        translation = -total / sec;
        new_data.push_back(data[i] + translation);
    }
    return new_data;
}

std::vector<double> fix_flip(const std::vector<double>& data, double diff_max = 200, int section_size = 100) {
    std::vector<double> new_data;
    if (data.empty()) {
        return new_data;
    }
    new_data.push_back(data[0]);
    for (size_t i = 1; i < data.size(); i++) {
        int window_size = std::min(section_size, static_cast<int>(new_data.size()));
        double sum_window = 0;
        for (int j = new_data.size() - window_size; j < new_data.size(); j++) {
            sum_window += new_data[j];
        }
        double average = sum_window / window_size;
        double diff = data[i] - average;
        if (diff > diff_max) {
            new_data.push_back(data[i] - 360);
        } else if (diff < -diff_max) {
            new_data.push_back(data[i] + 360);
        } else {
            new_data.push_back(data[i]);
        }
    }
    return new_data;
}

void strip_data(std::vector<double>& data) {
    while (!data.empty() && data.front() == 0) {
        data.erase(data.begin());
    }
    while (!data.empty() && data.back() == 0) {
        data.pop_back();
    }
}

void apply_preprocess(std::vector<json>& sensor_data, 
                      const std::function<std::vector<double>(const std::vector<double>&)>& f) {
    if (sensor_data.empty()) {
        return;
    }
    size_t numSensors = sensor_data[0]["sensors"].size();
    
    std::vector<std::string> angle_names;
    for (auto& item : sensor_data[0]["sensors"][0]["euler"].items()) {
        angle_names.push_back(item.key());
    }
    
    // For each sensor and each angle, extract the time-series, process it, then update.
    for (size_t si = 0; si < numSensors; si++) {
        for (const auto& angle : angle_names) {
            std::vector<double> data;
            data.reserve(sensor_data.size());
            for (auto& snapshot : sensor_data) {
                data.push_back(snapshot["sensors"][si]["euler"][angle].get<double>());
            }
            std::vector<double> new_data = f(data);

            for (size_t i = 0; i < sensor_data.size(); i++) {
                sensor_data[i]["sensors"][si]["euler"][angle] = new_data[i];
            }
        }
    }
}

void process_file(const std::string& file_path, const std::string& output_path) {
    std::vector<json> sensor_data;
    std::ifstream infile(file_path);
    if (!infile) {
        std::cerr << "Error opening input file: " << file_path << std::endl;
        std::exit(EXIT_FAILURE);
    }
    
    std::string line;
    while (std::getline(infile, line)) {
        if (!line.empty()) {
            try {
                sensor_data.push_back(json::parse(line));
            } catch (const std::exception& e) {
                std::cerr << "Error parsing JSON: " << e.what() << std::endl;
            }
        }
    }
    infile.close();
    
    apply_preprocess(sensor_data, [](const std::vector<double>& data) {
        return fix_flip(data);  
    });
    apply_preprocess(sensor_data, [](const std::vector<double>& data) {
        return normalize(data); 
    });
    
    
    // Write processed sensor data back to the output file (one JSON per line).
    std::ofstream outfile(output_path);
    if (!outfile) {
        std::cerr << "Error opening output file: " << output_path << std::endl;
        std::exit(EXIT_FAILURE);
    }
    for (const auto& snapshot : sensor_data) {
        outfile << snapshot.dump() << "\n";
    }
    outfile.close();
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_file_path> <output_file_path>\n";
        return EXIT_FAILURE;
    }
    
    std::string file_path = argv[1];
    std::string output_path = argv[2];
    
    process_file(file_path, output_path);
    
    return EXIT_SUCCESS;
}
