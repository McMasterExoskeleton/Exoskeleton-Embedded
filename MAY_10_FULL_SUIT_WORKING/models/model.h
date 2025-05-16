#pragma once

#include <torch/torch.h>
#include <torch/script.h>
#include <vector>

std::vector<float> predict_joint_angles(torch::jit::script::Module& model, const std::vector<std::vector<float>>& last_30_timesteps);
