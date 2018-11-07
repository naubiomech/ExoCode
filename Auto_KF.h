#ifndef AUTO_KF_HEADER
#define AUTO_KF_HEADER
#include "Utils.h"

void Auto_KF_motor_Late_stance(RunningAverage* error_average, double pid_setpoint, double motor_input);
double Auto_KF_motor_Swing(RunningAverage* error_average, double KF, Clamp* kf_clamp);

#endif
