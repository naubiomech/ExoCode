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
  Joint(JointPins* joint_pins);
  JointReport* generateReport();
  void fillReport(JointReport* report);

  void measureError();
  bool hasErrored();
  bool applyTorque();
  void updateSetpoint(int stadte);
  void scaleSetpointDifference();
  void sigmoidCurveSetpoint(int state);
  void setToZero();
  void adjustControl(int state, int state_old, int FSR_baseline_FLAG);
  void resetStartingParameters();
  void takeBaseline(int state, int state_old, int* FSR_baseline_FLAG);
  void adjustShapingForTime(double planterTime);
  void setTorqueScalar(double scalar);
  void updateMotorOutput(double FSR_percentage, double max_FSR_percentage);
  void updateKFPIDError();
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
