#ifndef SHAPING_FUNCTIONS_HEADER
#define SHAPING_FUNCTIONS_HEADER
#include "Utils.hpp"

class ShapingFunction{
private:
  int iteration_count = 0;
  int iteration_threshold = 0;
  int next_iteration_threshold = 0;

  double exp_mult = 0;

  Timer* recharge_timer = new Timer();
  void beginCurve();
  bool isCurveDone();
public:
  double shape(double desired_value, double current_value, double starting_value);
  double getPIDSetpoint(double newPIDSetpoint, double oldPIDSetpoint, double currentPIDSetpoint);
  void setIterationCount(double iterations);
  double getIterationCount();
};

#endif
