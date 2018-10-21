#ifndef SHAPING_FUNCTIONS_HEADER
#define SHAPING_FUNCTIONS_HEADER

class ShapingFunction{
private:
  int iteration_count = 0;
  int iteration_threshold = 0;

  double exp_mult = 0;
  int iter_late_stance = 0;
  int iter_swing = 0;
public:
  double getPIDSetpoint(double newPIDSetpoint, double oldPIDSetpoint, double currentPIDSetpoint, int state);
  void setIterationCount(int whichState, double n);
  double getIterationCount(int whichState);
};

#endif
