#ifndef CONTROL_ALGORITHMS_HEADER
#define CONTROL_ALGORITHMS_HEADER
#include "Utils.h"

enum ControlAlgorithm {zero_torque,bang_bang, unknown_control, porportional,pivot_porportional};

double getControlAlgorithmSetpoint(ControlAlgorithm control_algorithm, double desired_setpoint,
                                   Clamp* setpoint_clamp, double FSRatio, double Max_FSRatio);

#endif
