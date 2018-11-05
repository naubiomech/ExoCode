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
int Ctrl_ADJ_trigger_dorsi_start(){

  leg_steps->count_plant++; // you have accomplished a step
  // start dorsiflexion
  // calculate plantarflexion
  leg_steps->dorsi_time = millis();
  leg_steps->plant_time = millis() - (leg_steps->plant_time);

  leg_steps->plant_mean =
    take_baseline_get_mean(leg_steps->plant_time, leg_steps->plant_mean,
                           p_steps, leg_steps->plant_time_averager);


  if (motor_steps->flag_N3_adjustment_time) {
    N3 = round((leg_steps->plant_mean) * motor_steps->perc);
    N3 = min(500, max(4, N3));
  }
}

int Ctrl_ADJ_trigger_planter_start(){
  leg_steps->plant_time = millis(); // start the plantarflexion
  leg_steps->dorsi_time = millis() - (leg_steps->dorsi_time); // calculate the dorsiflexion that has just finished

}

int Ctrl_ADJ_dorsi(){

  if (taking_baseline) {
    *p_Setpoint_Ankle_Pctrl = 0;
    *p_Setpoint_Ankle = 0;
    return N3;
  }

}

int Ctrl_ADJ_planter(){

  if (taking_baseline) {
    *p_Setpoint_Ankle_Pctrl = 0;
    *p_Setpoint_Ankle = 0;
    return N3;
  }

  if (FSR_steps->curr_voltage > FSR_steps->peak_voltage)
    FSR_steps->peak_voltage =  FSR_steps->curr_voltage;

  *p_FSRatio = fabs(FSR_steps->curr_voltage / leg_steps->plant_peak_mean);
  if (*p_FSRatio > (*p_Max_FSRatio))
    (*p_Max_FSRatio) = *p_FSRatio;



  if (flag_torque_time_volt != 0){
    *p_Setpoint_Ankle_Pctrl = Ctrl_ADJ_planter_general(motor_steps, &p_FSRatio, &p_Max_FSRatio, flag_torque_time_volt);
    if (flag_torque_time_volt == 1){
      N3 = 1;
    }
    return N3;
  }
}
