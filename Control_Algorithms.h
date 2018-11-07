#ifndef CONTROL_ALGORITHMS_HEADER
#define CONTROL_ALGORITHMS_HEADER
#include "Utils.h"

double applyControlAlgorithm(int control_algorithm, double desired_setpoint,
                             Clamp* setpoint_clamp, double FSRatio, double Max_FSRatio);

#endif
