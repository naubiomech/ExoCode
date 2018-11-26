#ifndef JOINT_HEADER
#define JOINT_HEADER

#include "Motor.hpp"
#include "TorqueSensor.hpp"
#include "Report.hpp"
#include "Pins.hpp"
#include "Control_Module.hpp"

class Joint{
private:
  Motor* motor;
  TorqueSensor* torque_sensor;
  ControlModule* controller;

  double motor_output = 0;

  void fillLocalReport(JointReport* report);
public:
  Joint(Motor* motor, TorqueSensor* torque_sensor);
  JointReport* generateReport();
  void fillReport(JointReport* report);

  void measureError();
  bool hasErrored();
  bool applyTorque();
  void setToZero();
  void resetStartingParameters();
  void adjustShapingForTime(double planterTime);
  void updateMotorOutput(double FSR_percentage, double max_FSR_percentage);
  void changeControl(StateID state_id);
  void applyAutoKF();
  double getTorque();
  void measureTorque();
  void setSign(int sign);
  void startTorqueCalibration();
  void updateTorqueCalibration();
  void endTorqueCalibration();
};

#endif
