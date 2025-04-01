#include "StateParameters.h"
#include <cmath>

TorqueController::ModelParameters StateParameters::initializeModelParams()
{
  TorqueController::ModelParameters params = {
      .m1 = 5.65, .m2 = 3.48, .L1 = 0.41, .L2 = 0.4879, .r1 = 0.17, .r2 = 0.1892, .I1 = 0.0648, .I2 = 0.0107, .g = 9.81};
  return params;
}
