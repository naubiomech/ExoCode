#ifndef LEG_HEADER
#define LEG_HEADER
const int dim_FSR = 30;
const int dim = 5;

#include <i2c_t3.h>
#include "PID_v2.h"
#include "States.hpp"
#include "Joint.hpp"
#include "FSR.hpp"
#include "Report.hpp"
#include "IMU.hpp"
#include "Linked_List.hpp"

class Leg {
private:
  void applyControlAlgorithm();
  void startIncrementalActivation();
  bool isSteadyState();
  void fillLocalReport(LegReport* report);
  void measureIMUs();
  void adjustControl();
  void updateMotorSetpoints();

  LinkedList<Joint*> joints;
  LinkedList<FSRGroup*> fsrs;
  LinkedList<IMU*> imus;
  FSRGroup* foot_fsrs;

  bool set_motors_to_zero_torque;
  int step_count;
  int increment_activation_starting_step = 0;

  State* state;

public:
  Leg(State* states, LinkedList<Joint*> joints, LinkedList<FSRGroup*> fsrs, LinkedList<IMU*> imus);
  void measureSensors();
  bool checkMotorErrors();
  void attemptCalibration();
  void adjustSetpoint();
  void runAutoKF();
  void incrementStepCount();
  bool applyTorque();
  void applyStateMachine();
  void setZeroIfSteadyState();
  void resetStartingParameters();
  void autoKF();
  void adjustJointSetpoints();
  void startTorqueCalibration();
  void updateTorqueCalibration();
  void endTorqueCalibration();
  void calibrateFSRs();
  void setZeroIfNecessary();
  void changeState();
  void resetFSRMaxes();
  void adjustShapingForTime(double time);
  bool hasStateChanged(boolean foot_on_ground);
  bool determine_foot_on_ground();
  void applyControl();
  void setToZero();
  void calibrateIMUs();
  void setSign(int sign);
  LegReport* generateReport();
  void fillReport(LegReport* report);
  void setLegSign(int sign);
  void changeJointControl(StateID state_id);
};
#endif
