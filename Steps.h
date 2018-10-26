#ifndef STEPS_HEADER
#define STEPS_HEADER
#include "Parameters.h"

class Steps{
public:
  // Leg
  FixedAverage* plant_time_averager = new FixedAverage(n_step_baseline);
  FixedAverage* peak_time_averager = new FixedAverage(n_step_baseline);
  FixedAverage* dorsi_time_averager = new FixedAverage(n_step_baseline);

  double dorsi_time;
  double plant_time;

  double dorsi_mean;
  double plant_mean;
  double plant_peak_mean;

  int count_plant;
  int count_plant_base;

  bool flag_start_plant = false;
  // Motor
  Clamp* setpoint_clamp = new Clamp(Min_Prop, Max_Prop);
  bool flag_N3_adjustment_time = false;
  bool torque_adj = false;
  double Setpoint; // Desired setpoint
  double perc_l = 0.5;
  // FSR
  double curr_voltage;
  double fsr_percent_thresh_Toe = 0.9;
  double peak;
  // Maybe delete
  bool flag_take_baseline = false;
  double plant_mean_base;
  double voltage_peak_ref;
};

#endif
