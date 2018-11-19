#include <Arduino.h>
#include "Control_Algorithms.hpp"
#include "Parameters.hpp"


double ControlAlgorithm::clamp_setpoint(double raw_setpoint){
  if(raw_setpoint == 0){
    return 0;
  }

  double setpoint_sign = fabs(raw_setpoint) / raw_setpoint;
  double setpoint = setpoint_sign * setpoint_clamp->clamp(fabs(raw_setpoint));
  return setpoint;
}


double ZeroTorqueControl::getSetpoint(double fsr_percentage, double fsr_max_percentage){
  return 0;
}

ControlAlgorithmType ZeroTorqueControl::getType(){
  return zero_torque;
}

double BangBangControl::getSetpoint(double fsr_percentage, double fsr_max_percentage){
  double new_setpoint = desired_setpoint;
  return clamp_setpoint(new_setpoint);
}

ControlAlgorithmType BangBangControl::getType(){
  return bang_bang;
}

double BalanceControl::getSetpoint(double fsr_percentage, double fsr_max_percentage){
  // TODO implement balance control
  return 0;
}

ControlAlgorithmType BalanceControl::getType(){
  return balance_control;
}

double ProportionalControl::getSetpoint(double fsr_percentage, double fsr_max_percentage){
  double new_setpoint = desired_setpoint * gain * fsr_percentage;
  return clamp_setpoint(new_setpoint);
}

ControlAlgorithmType ProportionalControl::getType(){
  return proportional;
}

double ProportionalPivotControl::getSetpoint(double fsr_percentage, double fsr_max_percentage){
  double new_setpoint = (desired_setpoint) *
    (p_prop[0] * pow(fsr_percentage, 2) + p_prop[1] * (fsr_percentage) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2]) ;
  return clamp_setpoint(new_setpoint);
}

ControlAlgorithmType ProportionalPivotControl::getType(){
  return pivot_proportional;
}
