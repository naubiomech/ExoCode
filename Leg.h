#ifndef LEG_HEADER
#define LEG_HEADER
const int dim_FSR = 30;
const int dim = 5;

#include "PID_v2.h"
#include "Torque_Speed_ADJ.h"
#include "Motor.h"
#include "FSR.h"

class Leg {
public:
  Leg();
  void initialize();
  void measureSensors();
  bool checkMotorErrors();
  void takeFSRBaseline();
  bool applyTorque();
  void applyStateMachine();
  void setZeroIfSteadyState();
  void resetStartingParameters();
  void autoKF();
  // ---------
  // Leg
  Motor* ankle_motor;
  FSRGroup* foot_fsrs;


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

  // TODO: Make local variable
  long sig_time = 0;

  long sig_time_old = 0;

  // TODO: Make local variables
  int n_iter, N_step;

  // TODO: Find a better way to track state changing
  // TODO: Give names that relate to state
  int state = SWING;
  int state_old = SWING;
  int state_count_13 = 0;
  int state_count_31 = 0;

  double start_time = 0;

  steps* p_steps;

  boolean sigm_done;

  // ------------
};

void initialize_leg(Leg* leg);
void initialize_left_leg(Leg* left_leg);
void initialize_right_leg(Leg* right_leg);
#endif
