#ifndef TORQUE_SPEED_ADJ_HEADER
#define TORQUE_SPEED_ADJ_HEADER
#include "Steps.h"

// Stuctur in order to have all the needed values to adjust the torque or
// the shaping as a function of the force applied or of the speed respectively
int take_baseline(int R_state_l, int R_state_old_l, Steps* p_steps_l, int* p_flag_take_baseline_l);

double Ctrl_ADJ(int R_state_l, int R_state_old_l, Steps* p_steps_l, double N3_l, double New_PID_Setpoint_l,
                double* p_Setpoint_Ankle_l, double * p_Setpoint_Ankle_Pctrl_l, int flag_torque_time_volt_l,
                double prop_gain_l, double taking_baseline_l, double *p_FSR_Ratio, double* p_Max_FSR_Ratio);

#endif
