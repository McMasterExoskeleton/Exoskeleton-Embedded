#include <torch/torch.h>
#include <torch/script.h>
#include <vector>
#include <iostream>

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

/**
 * Predicts the next set of joint angles using the last 30 timesteps.
 * 
 * @param model A reference to the TorchScript model.
 * @param last_30_timesteps A vector of 30 timesteps, each timestep being a 6-float vector.
 *                          Shape: [30][6] = total 180 values in row-major format.
 * @return A std::vector<float> containing 6 predicted joint angles.
 */
std::vector<float> predict_joint_angles(torch::jit::script::Module& model, const std::vector<std::vector<float>>& last_30_timesteps) {
    if (last_30_timesteps.size() != 30) {
        throw std::invalid_argument("Expected 30 timesteps");
    }

    at::Tensor input_tensor = torch::zeros({1, 30, 6});
    for (int t = 0; t < 30; ++t) {
        if (last_30_timesteps[t].size() != 6) {
            throw std::invalid_argument("Each timestep must have 6 joint angles");
        }
        for (int j = 0; j < 6; ++j) {
            float normalized = (last_30_timesteps[t][j] - mean[j]) / scale[j];
            input_tensor[0][t][j] = normalized;
        }
    }

    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input_tensor);

    at::Tensor output = model.forward(inputs).toTensor();

    std::vector<float> denormalized_output(6);
    for (int i = 0; i < 6; ++i) {
        float val = output[0][i].item<float>();
        denormalized_output[i] = val * scale[i] + mean[i];
    }
    std::cout << "Predicted joint angles: ";
    for (const auto& angle : denormalized_output) {
        std::cout << angle << " ";
    }
    std::cout << std::endl;

    return denormalized_output;
}
