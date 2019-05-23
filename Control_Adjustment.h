// Stuctur in order to have all the needed values to adjust the torque or the shaping as a function of the force applied or of the speed respectively
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
  double Setpoint_Knee; // TN 5/9/19
  double plant_mean_base;
  double dorsi_mean_base;
  double voltage_peak_ref;
  double curr_voltage;
  double curr_voltage_Toe;
  double curr_voltage_Heel;
  double perc_l = 0.5;
  double fsr_percent_thresh_Toe = 0.1;
  double fsr_percent_thresh_Heel = 0.1; // TN 5/9/19
  double plant_peak_mean;
  double plant_peak_mean_Toe;
  double plant_peak_mean_Heel;
  double four_step_plant_peak[n_step_baseline] = {0};
  double four_step_plant_peak_Toe[n_step_baseline] = {0};
  double four_step_plant_peak_Heel[n_step_baseline] = {0};
  double plant_peak_mean_temp;
  double plant_peak_mean_temp_Toe;
  double plant_peak_mean_temp_Heel;
  double peak;
  double peak_Toe;
  double peak_Heel;

  double fsr_Toe;
  double fsr_Heel;  // TN 5/17/19
  double torque_average;
  double torque_average_K; // TN 5/9/19
} steps;

steps val_L;
steps val_R;
