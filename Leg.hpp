#ifndef LEG_HEADER
#define LEG_HEADER
const int dim_FSR = 30;
const int dim = 5;

#include <i2c_t3.h>
#include "PID_v2.h"
#include "States.hpp"
#include "Joint.hpp"
#include "FSR.hpp"
#include "Phase.hpp"
#include "Pins.hpp"
#include "Report.hpp"
#include "IMU.hpp"

class Leg {
private:
  void changeState(int new_state, int old_state);
  void changePhase(int new_phase);
  void applyControlAlgorithm();
  void applyPlanterControlAlgorithm();
  void applyDorsiControlAlgorithm();
  bool determinePhaseChange(int new_state, int old_state);
  void triggerPlanterPhase();
  void triggerDorsiPhase();
  int takeBaselineTriggerDorsiStart();
  int takeBaselineTriggerPlanterStart();
  void triggerSwing();
  void triggerLateStance();
  void updateIncrementalActivation();
  void startIncrementalActivation();
  bool isSteadyState();
  void trigger_planter_phase();
  void trigger_dorsi_phase();
  void fillLocalReport(LegReport* report);
  void measureIMUs();

  bool set_motors_to_zero_torque;

  Joint** joints;
  int joint_count;
  FSRGroup** fsrs;
  int fsr_group_count;
  IMU** imus;
  int imu_count;
  FSRGroup* foot_fsrs;

  Phase* current_phase;
  double last_phase_time;

  int step_count;

  Timer* state_time = new Timer();

  int increment_activation_starting_step = 0;

  // TODO Check usability of all variables below line
  // ------------

  Threshold* swing_state_threshold = new Threshold(false, 0.5, state_counter_th);

  double Prop_Gain = 1;

  double stateTimerCount;
  double flag_1 = 0;
  double time_old_state;

  int state = SWING;
  int state_old = SWING;

public:
  Leg(LegPins* legPins);
  void measureSensors();
  bool checkMotorErrors();
  void resetFSRMaxes();
  void updateFSRMaxes();
  void adjustSetpoint();
  void runAutoKF();
  void adjustShapingForTime(double time);
  void incrementStepCount();
  void takeFSRBaseline();
  bool applyTorque();
  void applyStateMachine();
  void setZeroIfSteadyState();
  void resetStartingParameters();
  void autoKF();
  void startTorqueCalibration();
  void updateTorqueCalibration();
  void endTorqueCalibration();
  void calibrateFSRs();
  int determineState(boolean foot_on_ground);
  bool determine_foot_on_ground();
  void adjustControl();
  void setToZero();
  void calibrateIMUs();
  void setSign(int sign);
  LegReport* generateReport();
  void fillReport(LegReport* report);
  void setLegSign(int sign);
};
#endif
