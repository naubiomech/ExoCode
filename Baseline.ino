void take_baseline_plant(FSR_Steps* fsr_steps){

  fsr_steps->max_fsr_voltage->update(steps->currVoltage);

}

void take_baseline_dorsi(Steps* steps){


}

int take_baseline_trigger_plant_start(Steps* steps){
  leg_steps->plant_timer->reset();
  fsr_steps->max_fsr_voltage->reset();

  double dorsi_time = leg_steps->dorsi_timer->lap();
  leg_steps->dorsi_mean = leg_steps->dorsi_time_averager->updateAverage(dorsi_time);

  return 1;
}

int take_baseline_trigger_dorsi_start(FSR_Steps* steps){
  leg_steps->dorsi_timer->reset();

  double plant_time = leg_steps->plant_timer->lap();
  leg_steps->plant_mean = leg_steps->plant_time_averager->updateAverage(plant_time);

  double plant_fsr_peak = fsr_steps->max_fsr_voltage->getMax();
  fsr_steps->plant_peak_mean = fsr_steps->plant_peak_averager->updateAverage(plant_fsr_peak);

  if (leg_steps->count_plant_base >= n_step_baseline){
    leg_steps->count_plant_base = 0;

    return 0;
  }
  return 1;
}
