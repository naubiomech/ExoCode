#ifndef SHAPING_FUNCTIONS_HEADER
#define SHAPING_FUNCTIONS_HEADER
#include "Leg.h"

double Change_PID_Setpoint_Sigm(double New_PID_Setpoint_l, double Current_PID_Setpoint, double Old_PID_Setpoint_l,
                                double Ts_l, double exp_mult_l, int n_iter_l, int N_l);

void PID_Sigm_Curve(Leg* leg);

void PID_Sigm_Curve_RL();

void PID_Sigm_Curve_LL();

#endif
