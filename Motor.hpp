#ifndef MOTOR_HEADER
#define MOTOR_HEADER
#include "Parameters.hpp"
#include "Utils.hpp"
#include "Shaping_Functions.hpp"
#include "Control_Algorithms.hpp"
#include "Pins.hpp"
#include "Report.hpp"
#include <PID_v2.h>

class Motor{
private:
  void fillLocalReport(MotorReport* report);

  unsigned int motor_pin;
  unsigned int motor_error_pin;

  bool in_error_state;

  double pid_setpoint = 0;
  double pid_input = 0;
  double pid_output = 0;

  PID* pid;

  RunningAverage* pid_avg_err;
  Clamp* kf_clamp;

  double KF = 1;

  Clamp* imu_clamp;

  double setpoint;
  double previous_setpoint = 0;

  double new_pid_setpoint = 0.0;
  double old_pid_setpoint = 0.0;

  double zero_torque_reference = ZERO_TORQUE_REFERENCE_DEFAULT;

  double prop_gain = 1;

  long sig_time_old = 0;
  ShapingFunction* shaping_function;
  double torque_scalar = 0;

  double iter_time_percentage = 0.5;

  Clamp* setpoint_clamp;
  double desired_setpoint = 0;
  ControlAlgorithm control_algorithm = zero_torque;
  ControlAlgorithm previous_control_algorithm = zero_torque;

  RunningAverage* error_average;

  bool adjust_shaping_for_time = false;
  int iter_late_stance = DEFAULT_ITER_LATE_STANCE;
  int iter_swing = DEFAULT_ITER_SWING;

public:
  Motor(MotorPins* motor_pins);
  void measureError();
  bool hasErrored();
  bool applyTorque(int state, double torque);
  void autoKF(int state);
  void writeToMotor(int value);
  void changeState(int state);
  void updateSetpoint(int state);
  void sigmoidCurveSetpoint(int state);
  void setToZero();
  void adjustControl(int state, int state_old, int FSR_baseline_FLAG);
  void resetStartingParameters();
  void takeBaseline(int state, int state_old, int* FSR_baseline_FLAG);
  void setControlAlgorithm(ControlAlgorithm control_algorithm);
  void adjustShapingForTime(double planterTime);
  void setTorqueScalar(double scalar);
  void applyPlanterControlAlgorithm(bool taking_baseline, double FSR_percentage, double max_FSR_percentage);
  void updateKFPIDError(double torque);
  void applyAutoKF();
  MotorReport generateReport();
  void fillReport(MotorReport* report);
};

#endif
