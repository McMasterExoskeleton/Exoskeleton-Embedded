#include "sensor_preprocessing.h"
#include <deque>
#include <numeric>
#include <cmath>

static void fix_flip(std::vector<float>& signal, float diff_max, int section_size) {
    if (signal.empty()) return;
    std::vector<float> new_data;
    new_data.reserve(signal.size());
    new_data.push_back(signal[0]);

    for (size_t i = 1; i < signal.size(); ++i) {
        int start = std::max(static_cast<int>(i) - section_size, 0);
        float mean_recent = std::accumulate(new_data.begin() + start, new_data.end(), 0.0f) / (i - start);
        float diff = signal[i] - mean_recent;

        if (diff > diff_max)
            new_data.push_back(signal[i] - 360.0f);
        else if (diff < -diff_max)
            new_data.push_back(signal[i] + 360.0f);
        else
            new_data.push_back(signal[i]);
    }

    signal = std::move(new_data);
}

static void normalize(std::vector<float>& signal, int section_size) {
    if (signal.empty()) return;

    std::deque<float> window;
    float total = 0.0f;
    std::vector<float> normalized;
    normalized.reserve(signal.size());

    for (size_t i = 0; i < signal.size(); ++i) {
        if (window.size() >= section_size) {
            total -= window.front();
            window.pop_front();
        }
        window.push_back(signal[i]);
        total += signal[i];
        float mean = total / static_cast<float>(window.size());
        normalized.push_back(signal[i] - mean);
    }

    signal = std::move(normalized);
}

void process_sensor_data(std::vector<std::vector<float>>& data, int section_size, float diff_max) {
    if (data.empty() || data[0].size() != 6) return;

    // Transpose data to [6][timesteps]
    std::vector<std::vector<float>> channels(6, std::vector<float>(data.size()));
    for (size_t t = 0; t < data.size(); ++t)
        for (size_t j = 0; j < 6; ++j)
            channels[j][t] = data[t][j];

    // Process each channel
    for (int j = 0; j < 6; ++j) {
        fix_flip(channels[j], diff_max, section_size);
        normalize(channels[j], section_size);
    }

    // Transpose back to [timesteps][6]
    for (size_t t = 0; t < data.size(); ++t)
        for (size_t j = 0; j < 6; ++j)
            data[t][j] = channels[j][t];
}
