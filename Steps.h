#ifndef STEPS_HEADER
#define STEPS_HEADER
#include "Parameters.h"

struct Steps{
	Leg_Steps leg_steps;
	Motor_Steps motor_steps;
	FSR_Steps fsr_steps;
};

struct Leg_Steps{
  // Leg
  FixedAverage* plant_time_averager = new FixedAverage(n_step_baseline);

  Timer* dorsi_timer = new Timer();
  Timer* plant_timer = new Timer();

  double plant_mean;
  double plant_peak_mean;

  int count_plant;

  bool flag_start_plant = false;
};

struct Motor_Steps{
  // Motor
  Clamp* setpoint_clamp = new Clamp(Min_Prop, Max_Prop);
  bool flag_N3_adjustment_time = false;
  bool torque_adj = false;
  double desired_setpoint;
  double perc_l = 0.5;
};

struct FSR_Steps{
  // FSR
  Max* max_fsr_ratio = new Max();
  Max* max_fsr_voltage = new Max();
  FixedAverage* plant_peak_fsr_averager = new FixedAverage(n_step_baseline);
                              double curr_voltage;
  double fsr_percent_thresh_Toe = 0.9;
  double peak_voltage;
};

#endif
