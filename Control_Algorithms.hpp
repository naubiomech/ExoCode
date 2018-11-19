#ifndef CONTROL_ALGORITHMS_HEADER
#define CONTROL_ALGORITHMS_HEADER
#include "Utils.hpp"
#include "Shaping_Functions.hpp"

enum ControlAlgorithmType {zero_torque, bang_bang, balance_control, proportional, pivot_proportional};

class ControlAlgorithm{
protected:
  double desired_setpoint;
  double gain;
  Clamp* setpoint_clamp;
  ShapingFunction* shaping_function;
  double clamp_setpoint(double raw_setpoint);
public:
  void setDesiredSetpoint(double setpoint);
  void setGain(double setpoint);
  virtual double getSetpoint(double fsr_percentage, double fsr_max_percentage) = 0;
  virtual ControlAlgorithmType getType() = 0;
};

class ZeroTorqueControl:public ControlAlgorithm{
public:
  double getSetpoint(double fsr_percentage, double fsr_max_percentage);
  ControlAlgorithmType getType();
};

class BangBangControl:public ControlAlgorithm{
public:
  double getSetpoint(double fsr_percentage, double fsr_max_percentage);
  ControlAlgorithmType getType();
};

class BalanceControl:public ControlAlgorithm{
public:
  double getSetpoint(double fsr_percentage, double fsr_max_percentage);
  ControlAlgorithmType getType();
};

class ProportionalControl:public ControlAlgorithm{
public:
  double getSetpoint(double fsr_percentage, double fsr_max_percentage);
  ControlAlgorithmType getType();
};

class ProportionalPivotControl:public ControlAlgorithm{
public:
  double getSetpoint(double fsr_percentage, double fsr_max_percentage);
  ControlAlgorithmType getType();
};

#endif
