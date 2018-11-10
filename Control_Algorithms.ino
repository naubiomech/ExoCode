#include "Control_Algorithms.h"

double clamp_setpoint(double raw_setpoint, Clamp* setpoint_clamp){
  if(raw_setpoint == 0){
    return 0;
  }

  double setpoint_sign = fabs(raw_setpoint) / raw_setpoint;
  double setpoint = setpoint_sign * setpoint_clamp->clamp(fabs(raw_setpoint));
  return setpoint;
}

double getSetpoint(ControlAlgorithm control_algorithm, double desired_setpoint,
                   Clamp* setpoint_clamp, double FSRatio, double Max_FSRatio, double prop_gain){
  double new_setpoint;

  switch (control_algorithm){
  case zero_torque:
    return 0;
    break;
  case bang_bang:
    break;
  case unknown_control:
    new_setpoint = (desired_setpoint ) *
      (p_prop[0] * pow((Max_FSRatio), 2) + p_prop[1] * (Max_FSRatio) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2]);
    break;
  case porportional:
    new_setpoint = (desired_setpoint ) * (prop_gain) * (FSRatio);
    break;
  case pivot_porportional:
    new_setpoint =(desired_setpoint ) *
      (p_prop[0] * pow(FSRatio, 2) + p_prop[1] * (FSRatio) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2]) ;
    break;
  }
  new_setpoint = clamp_setpoint(new_setpoint, setpoint_clamp);

  return new_setpoint;
}
