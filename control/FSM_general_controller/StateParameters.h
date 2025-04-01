#ifndef STATE_PARAMETERS_H
#define STATE_PARAMETERS_H

#include "../Joint_Estimator/JointEstimator.h"
#include "../Torque_Controller/TorqueController.h"
#include <fstream>

class StateParameters
{
public:
  // Constructor to initialize all parameters
  static TorqueController::ModelParameters initializeModelParams();
};

#endif