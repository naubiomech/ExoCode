#include "Control_Algorithms.h"
#include "Utils.h"

double applyControlAlgorithm(int control_algorithm, double desired_setpoint,
                             Clamp* setpoint_clamp, double FSRatio, double Max_FSRatio);
