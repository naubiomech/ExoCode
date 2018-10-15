#ifndef SHAPING_FUNCTIONS_HEADER
#define SHAPING_FUNCTIONS_HEADER

double calculatePIDSetpointSigm(double New_PID_Setpoint, double Old_PID_Setpoint,
								double Ts, double exp_mult, int n_iter, int N);
#endif
