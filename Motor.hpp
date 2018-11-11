#ifndef MOTOR_HEADER
#define MOTOR_HEADER
#include <PID_v2.h>

#include "Parameters.hpp"
#include "Utils.hpp"
#include "Shaping_Functions.hpp"
#include "Control_Algorithms.hpp"
#include "Pins.hpp"

class Motor{
private:
  unsigned int torque_sensor_pin;
  unsigned int motor_pin;
  unsigned int motor_error_pin;

  bool in_error_state;

  MovingAverage* torque_averager;

  double pid_setpoint, input, output;
  PID pid = PID(&input, &output, &pid_setpoint, PID_DEFAULTS[0], PID_DEFAULTS[1], PID_DEFAULTS[2], DIRECT);

  RunningAverage* pid_avg_err;
  Clamp* kf_clamp;

  double KF = 1;

  RunningAverage* torque_calibration_average;
  double torque_calibration_value = 0;
  Clamp* imu_clamp;

  int torque_address;

  double setpoint;
  double previous_setpoint = 0;

  double new_pid_setpoint = 0.0;
  double old_pid_setpoint = 0.0;

  double zero_torque_reference;

  double prop_gain = 1;

  long sig_time_old = 0;
  ShapingFunction* shaping_function;
  double torque_scalar = 0;

  double iter_time_percentage = 0.5;

  Clamp* setpoint_clamp;
  double desired_setpoint = 0;
  ControlAlgorithm control_algorithm = zero_torque;
  ControlAlgorithm previous_control_algorithm = zero_torque;

  double measureRawTorque();
  double measureRawCalibratedTorque();

  RunningAverage* error_average;

  bool adjust_shaping_for_time = false;
  int iter_late_stance = DEFAULT_ITER_LATE_STANCE;
  int iter_swing = DEFAULT_ITER_SWING;

public:
  Motor(MotorPins* motor_pins);
  double getTorque();
  void measureTorque();
  void measureError();
  bool hasErrored();
  bool applyTorque(int state);
  void autoKF(int state);
  void startTorqueCalibration();
  void updateTorqueCalibration();
  void endTorqueCalibration();
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
  void updateKFPIDError();
  void applyAutoKF();
};

#endif
