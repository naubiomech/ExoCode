#ifndef LEG_HEADER
#define LEG_HEADER
const int dim_FSR = 30;
const int dim = 5;

#include "PID_v2.h"
#include "Torque_Speed_ADJ.h"
#include "State_Machine.h"
#include "Motor.h"
#include "FSR.h"

class Leg {
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
  // ---------
  // Leg
  Motor** motors;
  int motor_count;
  FSRGroup* foot_fsrs;

  Threshold* swing_state_threshold = new Threshold(false, 0.5, state_counter_th);

  double Prop_Gain = 1;

  double stateTimerCount;
  double flag_1 = 0;
  double time_old_state;


  // TODO: Give proper name showing they relate to state
  double N3 = 500;
  double N2 = 4;
  double N1 = 4;

  double activate_in_3_steps = 0;
  double first_step = 1;
  double coef_in_3_steps = 0;
  double start_step = 0;
  double num_3_steps = 0;

  double coef_in_3_steps_Pctrl = 0;
  double store_N1 = 0;
  double set_2_zero = 0;
  double One_time_set_2_zero = 1;


  long sig_time_old = 0;

  int n_iter;

  // TODO: Find a better way to track state changing
  // TODO: Give names that relate to state
  int state = SWING;
  int state_old = SWING;

  double start_time = 0;

  steps* p_steps;

  boolean sigm_done;

  // ------------
};
#endif
