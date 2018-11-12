#ifndef LEG_HEADER
#define LEG_HEADER
const int dim_FSR = 30;
const int dim = 5;

#include "PID_v2.h"
#include "Torque_Speed_ADJ.h"

struct Leg {
  // Motors
  int torque_sensor_ankle_pin;
  int motor_ankle_pin;

  double sign = 1;

  double Tarray[dim] = {0};

  volatile double Average_Trq;

  unsigned int pin_err;

  double PID_Setpoint, Input, Output;
  PID balance_pid = PID(&Input, &Output, &PID_Setpoint, kp_balance, ki_balance, kd_balance, DIRECT);
  PID ankle_pid = PID(&Input, &Output, &PID_Setpoint, kp_ankle, ki_ankle, kd_ankle, DIRECT);
  // Auto_KF.h
  double ERR;
  double max_KF;
  double min_KF;
  int count_err;

  // TODO: Remove these in place of PID getters
  double kp_ankle = 700;
  double ki_ankle = 3;
  double kd_ankle = 0;
  double kp_balance = 60;
  double ki_balance = 0;
  double kd_balance = 20;
  double KF = 1;

  double torque_calibration_value = 0;
  int Vol;

  int torque_address;

  double Setpoint_Ankle, Setpoint_Ankle_Pctrl;
  double Previous_Setpoint_Ankle = 0;
  double Setpoint_earlyStance = 0.25 * Setpoint_Ankle;
  double Dorsi_Setpoint_Ankle;
  double Previous_Dorsi_Setpoint_Ankle;

  // TODO: Change these to local variables
  double exp_mult = 1500.0;
  boolean sigm_flag = true;
  boolean sigm_done;

  double New_PID_Setpoint = 0.0;
  double Old_PID_Setpoint = 0.0;

  double zero;
  // -------
  // FSRs

  double FSR_Average_array[dim_FSR] = {0};
  double Curr_FSR_Toe = 0;

  double FSR_Average_array_Heel[dim_FSR] = {0};
  double Curr_FSR_Heel = 0;

  volatile double FSR_Combined_Average;
  volatile double FSR_Toe_Average;
  volatile double FSR_Heel_Average;

  volatile double FSR_Toe_Balance_Baseline;
  volatile double FSR_Heel_Balance_Baseline;

  // Auto_KF.h
  double ERR;
  double max_KF;
  double min_KF;
  int count_err;
  bool auto_KF_update = true;

  double fsr_Combined_peak_ref;
  double Curr_Combined;
  double Curr_Toe;
  double Curr_Heel;

  // TODO: Give these names indicating they're pins
  unsigned int fsr_sense_Heel;
  unsigned int fsr_sense_Toe;

  double fsr_Heel = 0;
  double fsr_Toe = 0;
  double fsr_Heel_peak_ref = 0;
  double fsr_Toe_peak_ref = 0;

  double fsr_percent_thresh_Heel = 0.9;
  double fsr_percent_thresh_Toe = 0.9;

  double FSR_Ratio;
  double Max_FSR_Ratio;

  int FSR_baseline_FLAG = 0;

  int address_FSR;
  int baseline_address;
  double baseline_value;

  <<<<<<< HEAD
          // ---------
          // Leg
  =======
    // PID_and_Ctrl_Parameters.h
    double torque_calibration_value = 0;
  double T_act;
  int Vol;
  double kp = 700;
  double ki = 0;
  double kd = 3;
  double KF = 1;

  double PID_Setpoint, Input, Output;
  PID pid = PID(&Input, &Output, &PID_Setpoint, kp, ki, kd, DIRECT);

  double Setpoint_Ankle, Setpoint_Ankle_Pctrl;
  double Previous_Setpoint_Ankle = 0;
  double* p_Setpoint_Ankle = &Setpoint_Ankle;
  double* p_Setpoint_Ankle_Pctrl = &Setpoint_Ankle_Pctrl;
  double Setpoint_earlyStance = 0.25 * Setpoint_Ankle;
  double Dorsi_Setpoint_Ankle;
  double Previous_Dorsi_Setpoint_Ankle;
  double* p_Dorsi_Setpoint_Ankle = &Dorsi_Setpoint_Ankle;
  double* p_Previous_Dorsi_Setpoint_Ankle = &Previous_Dorsi_Setpoint_Ankle;
  >>>>>>> origin/GiammaUpdate


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
  int state_count_12 = 0;
  int state_count_21 = 0;
  int state_count_23 = 0;
  int state_count_32 = 0;

  double start_time = 0;

  steps* p_steps;

  // ------------
};

void initialize_leg(Leg* leg);
void initialize_left_leg(Leg* left_leg);
void initialize_right_leg(Leg* right_leg);
#endif
