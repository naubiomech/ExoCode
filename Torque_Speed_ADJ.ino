#include "Torque_Speed_ADJ.h"
#include "Steps.h"

double clamp_setpoint(double raw_setpoint, Clamp* setpoint_clamp){
  if(raw_setpoint == 0){
    return 0;
  }

  double setpoint_sign = fabs(raw_setpoint) / raw_setpoint;
  double setpoint = setpoint_sign * setpoint_clamp->clamp(fabs(raw_setpoint));
  return setpoint;
}

double Ctrl_ADJ_planter_general(Motor_Steps* motor_steps, double FSRatio,
                                double Max_FSRatio, int flag_torque_time_volt){
  double new_setpoint;

  if (flag_torque_time_volt == 2) {
    new_setpoint = (motor_steps->desired_setpoint ) * (prop_gain) * (FSRatio);
  } else if (flag_torque_time_volt == 3) {
    // Second version of pivot proportional Ctrl with polynomial law XXXXX QUI METTERE LEGGE
    new_setpoint =(motor_steps->desired_setpoint ) *
      (p_prop[0] * pow(FSRatio, 2) + p_prop[1] * (FSRatio) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2]) ;
  } else if (flag_torque_time_volt == 1) {
    new_setpoint = (motor_steps->desired_setpoint ) *
      (p_prop[0] * pow((Max_FSRatio), 2) + p_prop[1] * (Max_FSRatio) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2]);
  }

  new_setpoint = clamp_setpoint(new_setpoint, motor_steps->setpoint_clamp);

  return new_setpoint;
}

double Ctrl_ADJ_plantar(Steps* p_steps, int N3, double* p_FSRatio, double* p_Max_FSRatio,
                        int flag_torque_time_volt, double* p_Setpoint_Ankle_Pctrl, double* p_Setpoint_Ankle){


  if (flag_torque_time_volt != 0){
    *p_Setpoint_Ankle_Pctrl = Ctrl_ADJ_planter_general(motor_steps, &p_FSRatio, &p_Max_FSRatio, flag_torque_time_volt);
    if (flag_torque_time_volt == 1){
      N3 = 1;
    }
    return N3;
  }

  // Otherwise we need to calculate the time

  // Parameters for speed adaption
  if (leg_steps->flag_start_plant == false) // if it is true it means you started the step. Here I inizialize the parameters for speed adaption.
  {
    leg_steps->plant_time = millis(); // start the plantarflexion
    leg_steps->dorsi_time = millis() - (leg_steps->dorsi_time); // calculate the dorsiflexion that has just finished

    if (leg_steps->dorsi_time <= step_time_length / 4) // if <50ms probably it is noise
    {
      FSR_steps->peak_voltage = 0;
      leg_steps->flag_start_plant = false;
      return N3;
    }
  }
  leg_steps->flag_start_plant = true; // Parameters inizialized Start a step

  return N3;

}

double Ctrl_ADJ_dorsi(Steps* p_steps, double N3){

  if (leg_steps->flag_start_plant) { // If a step has started i.e. the states have passed from 1 or 2 to 3
    // if you transit from 3 to 1 plantar flexion is completed and start dorsiflexion

    leg_steps->count_plant++; // you have accomplished a step
    // start dorsiflexion
    // calculate plantarflexion
    leg_steps->dorsi_time = millis();
    leg_steps->plant_time = millis() - (leg_steps->plant_time);

    leg_steps->flag_start_plant = false;
    if (leg_steps->plant_time <= step_time_length) {
      return N3;
    }

    if (leg_steps->count_plant >= 2) {
      // this is the time window of the filter for the plantarflexion
      leg_steps->plant_mean =
        take_baseline_get_mean(leg_steps->plant_time, leg_steps->plant_mean,
                               p_steps, leg_steps->plant_time_averager);

    }

    if (motor_steps->flag_N3_adjustment_time) {
      N3 = round((leg_steps->plant_mean) * motor_steps->perc);
      N3 = min(500, max(4, N3));
    }
  }// end if flag_start_plant
  FSR_steps->peak_voltage = 0;
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
    if (FSR_steps->curr_voltage > FSR_steps->peak_voltage)
      FSR_steps->peak_voltage =  FSR_steps->curr_voltage;

    *p_FSRatio = fabs(FSR_steps->curr_voltage / leg_steps->plant_peak_mean);
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
