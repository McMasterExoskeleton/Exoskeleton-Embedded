#include "getSensorData.h"


using json = nlohmann::json;


std::vector<std::vector<float>> get_sensor_data(
    const std::string &filename,
    int chunk_index,
    int step_size
){ 
    std::ifstream file(filename);
    std::string line;
    std::vector<std::vector<float>> output;
    
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return {};
    }

    int start_line = chunk_index;
    int end_line = start_line + step_size;
    int current_line = 0;

    while (std::getline(file, line)) {
        if (current_line >= start_line && current_line < end_line) {
            json entry = json::parse(line);

            std::array<float, 6> rolls;
            int idx = 0;

            for (const auto &sensor : entry["sensors"]) {
                rolls[idx++] = sensor["euler"]["roll"];
            }

            if (idx == 6) output.push_back(std::vector<float>(rolls.begin(), rolls.end()));
        }

        current_line++;
        if (current_line >= end_line) break;
    }
    return output;
}

// int main(){
//     auto chunk = get_sensor_data("/home/dylan-exo/control_system/filtered_imu_data_treadmill_5min_1.9mph.json", 0, 30);

//     for (size_t i = 0; i < chunk.size(); ++i) {
//         std::cout << "step " << i << ": ";
//         for (float roll : chunk[i]) {
//             std::cout << roll << " ";
//         }
//         std::cout << std::endl;
//     }
// }