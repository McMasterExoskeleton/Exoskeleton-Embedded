#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <array>


/**
 * Parses roll angles from json .
 * 
 * @param filename Path to JSON file.
 * @param step_size Number of timesteps per chunk (i.e. read 30 lines per call).
 * @param chunk_index Index of the chunk to extract (index 1:0-30 entries. index 2: 31-60).
 * @return A vector of [step_size][6]
 */
std::vector<std::vector<float>> get_sensor_data(const std::string &filename, int chunk_index, int step_size);