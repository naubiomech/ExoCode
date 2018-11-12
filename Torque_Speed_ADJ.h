#ifndef TORQUE_SPEED_ADJ_HEADER
#define TORQUE_SPEED_ADJ_HEADER

// Stuctur in order to have all the needed values to adjust the torque or
// the shaping as a function of the force applied or of the speed respectively
typedef struct {
  int    n_steps;
  int    n_kf;
  int n_cycles;
  int prev_state;
  double dorsi_time;
  double plant_time;
  double dorsi_time_average;

  double four_step_dorsi_time[n_step_baseline] = {0};
  double four_step_plant_time[n_step_baseline] = {0};
  bool flag_take_average = false;
  bool flag_N3_adjustment_time = false;
  double dorsi_mean;
  double plant_mean;
  double dorsi_mean_old;
  double plant_mean_old;
  int count_plant;
  int count_plant_base;
  double torque_val;
  double torque_val_average;
  bool flag_start_plant = false;
  bool flag_take_baseline = false;
  bool torque_adj = false;
  double Setpoint;
  double plant_mean_base;
  double dorsi_mean_base;
  double voltage_peak_ref;
  double curr_voltage;
  double perc_l = 0.5;
  double fsr_percent_thresh_Toe = 0.9;
  double plant_peak_mean;
  double four_step_plant_peak[n_step_baseline] = {0};
  double plant_peak_mean_temp;
  double peak;
  double fsr_Toe;
  double torque_average;
} steps;

int take_baseline(int R_state_l, int R_state_old_l, steps* p_steps_l, int* p_flag_take_baseline_l);
double Ctrl_ADJ(int R_state_l, int R_state_old_l, steps* p_steps_l, double N3_l, double New_PID_Setpoint_l,
                double* p_Setpoint_Ankle_l, double * p_Setpoint_Ankle_Pctrl_l, int flag_torque_time_volt_l,
                double prop_gain_l, double taking_baseline_l, double *p_FSR_Ratio, double* p_Max_FSR_Ratio);

#endif
