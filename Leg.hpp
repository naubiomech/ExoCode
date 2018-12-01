#ifndef LEG_HEADER
#define LEG_HEADER
const int dim_FSR = 30;
const int dim = 5;

#include "Arduino.hpp"
#include "States.hpp"
#include "Joint.hpp"
#include "FSR.hpp"
#include "Report.hpp"
#include "IMU.hpp"
#include "Pot.hpp"
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
  LinkedList<Pot*> pots;
  FSRGroup* foot_fsrs;

  bool set_motors_to_zero_torque;
  int step_count;
  int increment_activation_starting_step;

  State* state;

public:
  Leg(State* states, LinkedList<Joint*>& joints, LinkedList<FSRGroup*>& fsrs,
      LinkedList<IMU*>& imus, LinkedList<Pot*>& pots);
  ~Leg();
  void measureSensors();
  void measurePots();
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
  bool hasStateChanged(bool foot_on_ground);
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
