#ifndef JOINT_HEADER
#define JOINT_HEADER

#include "Motor.hpp"
#include "TorqueSensor.hpp"
#include "Report.hpp"
#include "Pins.hpp"

class Joint{
private:
  Motor* motor;
  TorqueSensor* torque_sensor;
  ControlAlgorithm* control_algorithm;

  double pid_setpoint = 0;
  double pid_input = 0;
  double pid_output = 0;

  PID* pid;

  Clamp* kf_clamp;

  double KF = 1;

  Clamp* imu_clamp;

  double setpoint;
  double previous_setpoint = 0;

  double new_pid_setpoint = 0.0;
  double old_pid_setpoint = 0.0;

  ShapingFunction* shaping_function;
  double torque_scalar = 0;

  double iter_time_percentage = 0.5;

  Clamp* setpoint_clamp;
  double desired_setpoint = 0;

  bool adjust_shaping_for_time = false;
  RunningAverage* error_average;

  void fillLocalReport(JointReport* report);
public:
  Joint(JointPins* joint_pins);
  JointReport* generateReport();
  void fillReport(JointReport* report);

  void measureError();
  bool hasErrored();
  bool applyTorque(int state);
  void updateSetpoint(int stadte);
  void scaleSetpointDifference();
  void sigmoidCurveSetpoint(int state);
  void setToZero();
  void adjustControl(int state, int state_old, int FSR_baseline_FLAG);
  void resetStartingParameters();
  void takeBaseline(int state, int state_old, int* FSR_baseline_FLAG);
  void adjustShapingForTime(double planterTime);
  void setTorqueScalar(double scalar);
  void adjustSetpoint(double FSR_percentage, double max_FSR_percentage);
  void updateKFPIDError();
  void applyAutoKF();
  double getTorque();
  void measureTorque();
  void setSign(int sign);
  void startTorqueCalibration();
  void updateTorqueCalibration();
  void endTorqueCalibration();
};

#endif
