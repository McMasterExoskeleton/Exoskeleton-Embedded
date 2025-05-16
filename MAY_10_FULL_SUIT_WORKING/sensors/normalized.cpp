// ================= C++ Simulation (Updated with Full Inference Loop, Normalization, and Alignment) =================

#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>
#include <thread>
#include <chrono>
#include <ctime>

std::vector<float> mean = {
    -0.2551588541666669,
    -0.5622835937500013,
    -0.6163348958333331,
    -0.10561770833333331,
    2.1784476562499995,
    0.005826041666666451
};

std::vector<float> scale = {
    12.578133062788302,
    19.253724193808857,
    6.668231683716051,
    7.107068858678035,
    15.89329980993656,
    5.110401095770009
};

// Function to read data from the JSON file
nlohmann::json read_json_data(const std::string& file_name) {
    std::ifstream file(file_name);
    nlohmann::json data;
    file >> data;
    return data;
}

std::string get_current_timestamp() {
    auto now = std::time(nullptr);
    char buf[100];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", localtime(&now));
    return std::string(buf);
}

void log_message(const std::string& message) {
    std::cout << get_current_timestamp() << " - " << message << std::endl;
}

int main() {
    log_message("Loading model...");

    torch::jit::script::Module model;
    try {
        model = torch::jit::load("./model.pt");
        model.eval();
        log_message("Model loaded successfully.");
    } catch (const c10::Error& e) {
        log_message("Error loading the model: " + std::string(e.what()));
        return -1;
    }

    std::ifstream input_file("filtered_imu_data_treadmill_5min_1.9mph.json");
    std::ofstream output_file("/home/dylan-exo/data.txt");

    std::string line;
    std::vector<float> input_values;
    int line_count = 0;

    // Read all data into input_values
    while (std::getline(input_file, line)) {
        nlohmann::json sensor_data = nlohmann::json::parse(line);
        for (size_t i = 0; i < sensor_data["sensors"].size(); ++i) {
            float roll_angle = sensor_data["sensors"][i]["euler"]["roll"];
            input_values.push_back(roll_angle);
        }
    }

    size_t total_timesteps = input_values.size() / 6;
    size_t prediction_count = 0;

    for (size_t start_t = 0; start_t + 30 < total_timesteps; ++start_t) {
        // Construct normalized input tensor
        at::Tensor input_tensor = torch::zeros({1, 30, 6});
        for (int t = 0; t < 30; ++t) {
            for (int j = 0; j < 6; ++j) {
                float val = input_values[(start_t + t) * 6 + j];
                val = (val - mean[j]) / scale[j];
                input_tensor[0][t][j] = val;
            }
        }

        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(input_tensor);

        auto start = std::chrono::high_resolution_clock::now();
        at::Tensor output = model.forward(inputs).toTensor();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> inference_time = end - start;

        log_message("Inference time: " + std::to_string(inference_time.count()) + " ms");

        for (size_t i = 0; i < 6; ++i) {
            float predicted_roll = output[0][i].item<float>();
            predicted_roll = predicted_roll * scale[i] + mean[i];
            float actual_roll = input_values[(start_t + 30) * 6 + i];

            log_message("[" + std::to_string(prediction_count) + "] Predicted: " + std::to_string(predicted_roll) +
                        ", Actual: " + std::to_string(actual_roll));

            output_file << actual_roll << " " << predicted_roll << "\n";
        }

        prediction_count++;
    }

    log_message("Finished processing " + std::to_string(prediction_count) + " predictions.");
    input_file.close();
    output_file.close();

    return 0;
}
