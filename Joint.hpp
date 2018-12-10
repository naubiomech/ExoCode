#ifndef JOINT_HEADER
#define JOINT_HEADER

#include "Motor.hpp"
#include "TorqueSensor.hpp"
#include "Report.hpp"
#include "Port.hpp"
#include "Control_Module.hpp"
#include "Message.hpp"

class Joint{
private:
  Motor* motor;
  TorqueSensor* torque_sensor;
  ControlModule* controller;

  double motor_output;

  void fillLocalReport(JointReport* report);
public:
  Joint(ControlModule* controller, Motor* motor, TorqueSensor* torque_sensor);
  ~Joint();
  void processMessage(JointMessage* msg);
  JointReport* generateReport();
  void fillReport(JointReport* report);

  void measureError();
  bool hasErrored();
  bool applyTorque();
  void setToZero();
  void resetStartingParameters();
  void adjustShapingForTime(double planterTime);
  void updateMotorOutput(SensorReport* report);
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
