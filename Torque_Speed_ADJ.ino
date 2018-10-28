#include "Torque_Speed_ADJ.h"
#include "Steps.h"

int take_baseline_plantar(Steps* p_steps){

  if (p_steps->flag_start_plant == false) // if it is true it means you started the step. Here I inizialize the parameters for speed adaption.
  {
    p_steps->plant_time = millis(); // start the plantarflexion
    p_steps->dorsi_time = millis() - (p_steps->dorsi_time); // calculate the dorsiflexion that has just finished

    if (p_steps->dorsi_time <= step_timelength / 4) // if <50ms probably it is noise
    {
      p_steps->peak = 0;
      p_steps->flag_start_plant = false;
      Serial.println(" BASE Dorsi too short");
    } else {
      p_steps->flag_start_plant = true; // Parameters inizialized Start a step
      Serial.println(" BASE Start Plantar");
    }
  }
  return 0;
}

double take_baseline_get_mean_time(double time, double current_mean, Steps* p_steps, FixedAverage* averager){
  double mean = current_mean;
  if (((p_steps->count_plant_base) - 2) >= n_step_baseline) {
    mean = averager->updateAverage(time);
  } else {
    averager->updateAverage(time);
    mean = current_mean;
  }
  return mean;
}

int take_baseline_update_mean_time(Steps* p_steps){

  p_steps->plant_mean =
    take_baseline_get_mean_time(p_steps->plant_time, p_steps->plant_mean,
                                p_steps, p_steps->plant_time_averager);

  p_steps->plant_peak_mean = 0.9 *
    take_baseline_get_mean_time(p_steps->plant_peak_time, p_steps->plant_peak_mean,
                                p_steps, p_steps->plant_peak_time_averager);
}

int take_baseline_dorsi(Steps* p_steps){
  if (p_steps->flag_start_plant) {
    // If a step has started i.e. the states have passed from 1 or 2 to 3
    // if you transit from 3 to 1 plantar flexion is completed and start dorsiflexion

    // start dorsiflexion

    p_steps->dorsi_time = millis();
    // calculate plantarflexion
    p_steps->plant_time = millis() - (p_steps->plant_time);

    if (p_steps->plant_time <= step_time_length)
    {
      p_steps->flag_start_plant = false;
      return 0;
    } else {
      p_steps->flag_start_plant = false; // you have provided one plant
      p_steps->count_plant_base++; // you have accomplished a step
    }

    if ((p_steps->count_plant_base) >= 2) { // avoid the first step just to be sure

      take_baseline_update_mean_time();

      if (((p_steps->count_plant_base) - 2) >= n_step_baseline) {
        (p_steps->count_plant_base) = 0;
        return (p_steps->count_plant_base);

      } // return 1 activate a flag that stops the calc of the baseline
    }// end if count_plant>2

  }

  p_steps->peak = 0;
  return 0;
}

int take_baseline(int state, int state_old, Steps* p_steps) {


  if ((state == LATE_STANCE) && state_old == SWING) { // I am in plantarflexion
    // update the voltage peak
    if (p_steps->curr_voltage > p_steps->peak)
      p_steps->peak =  p_steps->curr_voltage;

    return take_baseline_plant(p_steps);
  }

  if ((state_old == 3) && (state == 1 || state == 2)) {
    return take_baseline_dorsi(p_steps);
  }

  return 0;
}

double clamp_setpoint(double raw_setpoint, Clamp* setpoint_clamp){
  if(raw_setpoint == 0){
    return 0;
  }

  double setpoint_sign = fabs(raw_setpoint) / raw_setpoint;
  double setpoint = setpoint_sign * setpoint_clamp->clamp(fabs(raw_setpoint));
  return setpoint;
}

double Ctrl_ADJ_plantar(Steps* p_steps, int N3, double* p_FSRatio, double* p_Max_FSRatio,
                        int flag_torque_time_volt, double* p_Setpoint_Ankle_Pctrl, double* p_Setpoint_Ankle){



  if (flag_torque_time_volt != 0){
    double new_setpoint;
    if (flag_torque_time_volt == 2) {
      new_setpoint = (p_steps->desired_setpoint ) * (prop_gain) * (*p_FSRatio);

    } else if (flag_torque_time_volt == 3) {
      // Second version of pivot proportional Ctrl with polynomial law XXXXX QUI METTERE LEGGE
      new_setpoint =(p_steps->desired_setpoint ) *
        (p_prop[0] * pow(*p_FSRatio, 2) + p_prop[1] * (*p_FSRatio) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2]) ;

    } else if (flag_torque_time_volt == 1) {

      new_setpoint = (p_steps->desired_setpoint ) *
        (p_prop[0] * pow((*p_Max_FSRatio), 2) + p_prop[1] * (*p_Max_FSRatio) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2]);

      N3 = 1;
    }
    new_setpoint = clamp_setpoint(new_setpoint, p_steps->setpoint_clamp);
    *p_Setpoint_Ankle_Pctrl = new_setpoint;
    return N3;
  }

  // Otherwise we need to calculate the time

  // Parameters for speed adaption
  if (p_steps->flag_start_plant == false) // if it is true it means you started the step. Here I inizialize the parameters for speed adaption.
  {
    p_steps->plant_time = millis(); // start the plantarflexion
    p_steps->dorsi_time = millis() - (p_steps->dorsi_time); // calculate the dorsiflexion that has just finished

    if (p_steps->dorsi_time <= step_time_length / 4) // if <50ms probably it is noise
    {
      p_steps->peak = 0;
      p_steps->flag_start_plant = false;
      return N3;
    }

    p_steps->flag_start_plant = true; // Parameters inizialized Start a step
  }


  // Torque adaption as a function of the speed or of the pressure force
  // If you want to adjust the torque and hence torque_adj = 1
  //-----------------add 11:49 6/19/18
  if (p_steps->torque_adj)
  {
    if (p_steps->plant_time <= step_time_length)
    {
      p_steps->peak = 0;
      p_steps->flag_start_plant = false;
      Serial.println(" TRQ ADJ plant time too short ");
      return N3;
    }


    // if you use plantar time as reference to increase the torque
    double new_setpoint;
    if (p_steps->flag_start_plant == true) {
      if (flag_torque_time_volt == 0) {
        // if you're going use the time as reference to increase also the torque
        new_setpoint = (p_steps->desired_setpoint ) * (1 / (fabs(p_steps->plant_mean)));
      } else if (flag_torque_time_volt == 1) { // If you use the volt or force value returned by the FSR sensors
        new_setpoint = (p_steps->desired_setpoint ) * (fabs(p_steps->peak / p_steps->plant_peak_mean));
      }
      new_setpoint = clamp_setpoint(new_setpoint, p_steps->setpoint_clamp);
      *p_Setpoint_Ankle = new_setpoint;
    }
  }// end torque adj
  return N3;

}

double Ctrl_ADJ_dorsi(Steps* p_steps, double N3){

  if (p_steps->flag_start_plant) { // If a step has started i.e. the states have passed from 1 or 2 to 3
    // if you transit from 3 to 1 plantar flexion is completed and start dorsiflexion

    p_steps->count_plant++; // you have accomplished a step

    // start dorsiflexion
    p_steps->dorsi_time = millis();

    // calculate plantarflexion
    p_steps->plant_time = millis() - (p_steps->plant_time);

    if (p_steps->plant_time <= step_time_length) {
      p_steps->flag_start_plant = false;
      return N3;
    }

    p_steps->flag_start_plant = false; // you have provided one step

    if (p_steps->count_plant >= 2) {
      // this is the time window of the filter for the plantarflexion
      p_steps->plant_mean =
        take_baseline_get_mean_time(p_steps->plant_time, p_steps->plant_mean,
                                    p_steps, p_steps->plant_time_averager);

      // Update mean value
      p_steps->plant_mean = p_steps->plant_time_averager->getAverage();
    }

    if (p_steps->flag_N3_adjustment_time) {
      N3 = round((p_steps->plant_mean) * p_steps->perc);
      N3 = min(500, max(4, N3));
    }
  }// end if flag_start_plant
  p_steps->peak = 0;
  p_Max_FSRatio = 0;
  return N3;
}

double Ctrl_ADJ(int state, int state_old, Steps* p_steps, double N3, double New_PID_Setpoint,
                double* p_Setpoint_Ankle, double * p_Setpoint_Ankle_Pctrl, int flag_torque_time_volt,
                double prop_gain, double taking_baseline, double *p_FSRatio, double* p_Max_FSRatio) {

  // Speed adjustment -> the smoothing parameters are updated as a function of the plantar time in order to modify the
  // shaping of the torque to the new step time.

  // It considers the average time of 4 steps. There's a filter of step_time_length on the plantarflexion time in order
  // to cut the noise the faster you go the more N3 decreases

  // Torque adjustment -> the torque set point is modified as a function of the voltage measured ,
  // the faster you go probably the bigger you hit the floor and hence the voltage increases.
  //It modifies the setpoint at each step.

  if (taking_baseline) {
    *p_Setpoint_Ankle_Pctrl = 0;
    *p_Setpoint_Ankle = 0;
    return N3;
  }

    // if you transit from state 1 to state 3 dorsiflexion is completed and start plantarflexion
  if ((state == LATE_STANCE) && (state_old == SWING)) {
    // update the voltage peak to update torque in case of Bang Bang ctrl
    if (p_steps->curr_voltage > p_steps->peak)
      p_steps->peak =  p_steps->curr_voltage;

    *p_FSRatio = fabs(p_steps->curr_voltage / p_steps->plant_peak_mean);
    if (*p_FSRatio > (*p_Max_FSRatio))
      (*p_Max_FSRatio) = *p_FSRatio;
    N3 = Ctrl_ADJ_plantar(p_steps, N3, p_FSRatio, p_Max_FSRatio,
                          flag_torque_time_volt, p_Setpoint_Ankle_Pctrl, p_Setpoint_Ankle);
  }

  if ((state == SWING) && (state_old == LATE_STANCE)) {
    N3 = Ctrl_ADJ_dorsi(p_steps, N3);
      // During the all dorsiflexion set the voltage peak to 0, probably we just need to do it one time
  }

  return N3;
  }
