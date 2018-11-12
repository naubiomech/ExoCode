#ifndef LEG_HEADER
#define LEG_HEADER
const int dim_FSR = 30;
const int dim = 5;

#include <i2c_t3.h>
#include "PID_v2.h"
#include "State_Machine.hpp"
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
  Phase determinePhase(int new_state, int old_state, Phase current_phase);
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

  bool set_motors_to_zero_torque;

  Joint** joints;
  int joint_count;
  FSRGroup** fsrs;
  int fsr_group_count;
  IMU** imus;
  int imu_count;
  FSRGroup* foot_fsrs;

  Phase current_phase;

  int planter_step_count_base;
  int planter_step_count;

  Timer* dorsi_timer = new Timer();
  Timer* planter_timer = new Timer();
  Timer* state_time = new Timer();

  MovingAverage* planter_time_averager = new MovingAverage(4);
  MovingAverage* dorsi_time_averager = new MovingAverage(4);

  double dorsi_mean;
  double planter_mean;

  int increment_activation_starting_step = 0;

  // TODO Check usability of all variables below line
  // ------------

  Threshold* swing_state_threshold = new Threshold(false, 0.5, state_counter_th);

  double Prop_Gain = 1;

  double stateTimerCount;
  double flag_1 = 0;
  double time_old_state;

  double activate_in_3_steps = 0;
  double first_step = 1;
  double start_step = 0;
  double num_3_steps = 0;

  double coef_in_3_steps_Pctrl = 0;
  double store_N1 = 0;
  double set_2_zero = 0;
  double One_time_set_2_zero = 1;

  int state = SWING;
  int state_old = SWING;

  double start_time = 0;

  int FSR_baseline_FLAG = 0;

public:
  Leg(LegPins* legPins);
  void measureSensors();
  bool checkMotorErrors();
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
  double getBalanceReference();
  int determineState(boolean foot_on_ground);
  bool determine_foot_on_ground();
  void adjustControl();
  void setToZero();
  LegReport* generateReport();
  void fillReport(LegReport* report);
};
#endif
