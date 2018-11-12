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

  void fillLocalReport(JointReport* report);
public:
  Joint(JointPins* joint_pins);
  JointReport generateReport();
  void fillReport(JointReport* report);

  void measureError();
  bool hasErrored();
  bool applyTorque(int state);
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
  void updateKFPIDError();
  void applyAutoKF();
  double getTorque();
  void measureTorque();
  void startTorqueCalibration();
  void updateTorqueCalibration();
  void endTorqueCalibration();
};

#endif
