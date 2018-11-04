#include "Steps.h"

int take_baseline_plantar(Leg_Steps* leg_steps, FSR_Steps* fsr_steps){

  if (leg_steps->flag_start_plant == false) // if it is true it means you started the step. Here I inizialize the parameters for speed adaption.
  {
    leg_steps->plant_timer->reset();
    leg_steps->dorsi_time = leg_steps->dorsi_timer->lap();

    if (leg_steps->dorsi_time <= step_timelength / 4) // if <50ms probably it is noise
    {
      FSR_steps->peak_voltage = 0;
      leg_steps->flag_start_plant = false;
      Serial.println(" BASE Dorsi too short");
    } else {
      leg_steps->flag_start_plant = true; // Parameters inizialized Start a step
      Serial.println(" BASE Start Plantar");
    }
  }
  return 0;
}

double take_baseline_get_mean(double value, FixedAverage* averager){
  double mean;
  mean = averager->updateAverage(value);
  return mean;
}

int take_baseline_update_means(Leg_Steps* leg_steps){

  leg_steps->plant_mean =
    take_baseline_get_mean(leg_steps->plant_time, leg_steps->plant_time_averager);

  FSR_steps->plant_peak_mean = 0.9 *
    take_baseline_get_mean(leg_steps->peak_voltage, leg_steps->plant_peak_fsr_averager);
}

int take_baseline_dorsi(Leg_Steps* leg_steps, FSR_Steps* FSR_steps){
  if (leg_steps->flag_start_plant) {
    // If a step has started i.e. the states have passed from 1 or 2 to 3
    // if you transit from 3 to 1 plantar flexion is completed and start dorsiflexion

    // start dorsiflexion

    leg_steps->dorsi_timer->reset();
    leg_steps->plant_time = leg_steps->plant_timer->lap();

    leg_steps->flag_start_plant = false;
    if (leg_steps->plant_time <= step_time_length) {
      return 0;
    }

    take_baseline_update_means(leg_steps);

  }

  FSR_steps->peak_voltage = 0;
  return 0;
}

int take_baseline(int state, int state_old, Steps* p_steps) {
  Leg_Steps* leg_steps = &(p_steps->leg_steps);
  Motor_Steps* motor_steps = &(p_steps->motor_steps);
  FSR_Steps* fsr_steps = &(p_steps->fsr_steps);

  if ((state == LATE_STANCE) && state_old == SWING) { // I am in plantarflexion
    // update the voltage peak
    if (FSR_steps->curr_voltage > FSR_steps->peak_voltage)
      FSR_steps->peak_voltage =  FSR_steps->curr_voltage;

    return take_baseline_plant(leg_steps, fsr_steps);
  }

  if ((state_old == 3) && (state == 1 || state == 2)) {
    take_baseline_dorsi(leg_steps);
    return 0;
  }

  return 0;
}
