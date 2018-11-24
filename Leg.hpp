#ifndef LEG_HEADER
#define LEG_HEADER
const int dim_FSR = 30;
const int dim = 5;

#include <i2c_t3.h>
#include "PID_v2.h"
#include "States.hpp"
#include "Joint.hpp"
#include "FSR.hpp"
#include "Pins.hpp"
#include "Report.hpp"
#include "IMU.hpp"

class Leg {
private:
  void applyControlAlgorithm();
  void startIncrementalActivation();
  bool isSteadyState();
  void fillLocalReport(LegReport* report);
  void measureIMUs();
  void adjustControl();
  void updateMotorSetpoints();

  Joint** joints;
  int joint_count;
  FSRGroup** fsrs;
  int fsr_group_count;
  IMU** imus;
  int imu_count;
  FSRGroup* foot_fsrs;

  bool set_motors_to_zero_torque;
  int step_count;
  int increment_activation_starting_step = 0;

  // TODO Check usability of all variables below line
  // ------------

  Threshold* swing_state_threshold = new Threshold(false, 0.5, state_counter_th);

  double Prop_Gain = 1;

  double stateTimerCount;
  double flag_1 = 0;
  double time_old_state;


  State* state;

public:
  Leg(LegPins* legPins);
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
