#include "ema_filter.h"


#ifndef FAULT_DETECTION
#define FAULT_DETECTION
/*
 * The fault detection is currently performed in tracking check. If the exo is tracking poorly,
 * then the torque sensor may be bad. The tunable parameters are included as defines below. The 
 * tracking_check() function performs the error checking. By applying an EMA filter on the setpoint
 * the torque response may be approximated. The error between this approximated value and the actual
 * torque response are computed, and passed through an EMA filter. The averaged error is used compared
 * to the tracking_thresh define and, if greater, an error is thrown. 
 */
 inline void turn_off_motors() {
  flag_motor_error_check = false;
  flag_auto_KF = false;
  digitalWrite(onoff, LOW);
 }

//Tracking error in N*m
#define TRACKING_THRESH       9.0
//Estimated torque alpha
#define EST_TRQ_ALPHA         0.1
//Error alpha
#define ERROR_ALPHA           0.005
//Tracking error rate in (N*m)/s
#define TRACKING_RATE_THRESH  1.5 * TRACKING_THRESH / CONTROL_TIME_STEP
//Max torque setpoint
#define MAX_TORQUE            35
//Max torque rate
#define MAX_TORQUE_RATE       MAX_TORQUE / CONTROL_TIME_STEP

//Protocols
inline void check_for_torque_faults(Leg* leg);
inline void tracking_check(Leg* leg);


void detect_faults() {
  check_for_torque_faults(right_leg);
  check_for_torque_faults(left_leg);
}

inline void check_for_torque_faults(Leg* leg) {
  tracking_check(leg);
  torque_check(leg);
}
  
inline void tracking_check(Leg* leg) {
  //Compare smoothed setpoint with measured torque
  static double filtered_sp_L = 0;
  static double filtered_sp_R = 0;
  static double track_error_L = 0;
  static double track_error_R = 0;
  static double filtered_error_L = 0;
  static double filtered_error_R = 0;

  double sign = (Control_Mode == 8) ? -1:1;

  if (leg == right_leg) {
    filtered_sp_R = ema_with_context(filtered_sp_R, right_leg->PID_Setpoint, EST_TRQ_ALPHA);
    double current_track_error = abs(filtered_sp_R - leg->Average_Trq);
    filtered_error_R = abs(ema_with_context(filtered_error_R, current_track_error, ERROR_ALPHA));
    double track_error_rate = abs((current_track_error - track_error_R) / CONTROL_TIME_STEP);
    if (((sign * filtered_error_R) > TRACKING_THRESH || (sign * track_error_rate) > TRACKING_RATE_THRESH) && stream) {
      if (sign * filtered_error_R > TRACKING_THRESH) r_state += 1;
      if (sign * track_error_rate > TRACKING_RATE_THRESH) r_fsr += 1;
      turn_off_motors();
    }
    track_error_R = filtered_error_R;
    r_set = filtered_error_R; 
  } 
  else {
    filtered_sp_L = ema_with_context(filtered_sp_L, left_leg->PID_Setpoint, EST_TRQ_ALPHA);
    double current_track_error = abs(filtered_sp_L - leg->Average_Trq);
    filtered_error_L = abs(ema_with_context(filtered_error_L, current_track_error, ERROR_ALPHA));
    double track_error_rate = abs((filtered_error_L - track_error_L) / CONTROL_TIME_STEP);
    if (((sign * filtered_error_L) > TRACKING_THRESH || (sign * track_error_rate) > TRACKING_RATE_THRESH) && stream) {
      if (sign * filtered_error_L > TRACKING_THRESH) l_state += 1;
      if (sign * track_error_rate > TRACKING_RATE_THRESH) l_fsr += 1;
      turn_off_motors();
    }
    track_error_L = filtered_error_L;
    l_set = filtered_error_L;
  }
}

void torque_check(Leg* leg) {
  double abs_trq = abs(leg->Average_Trq);
  /* Absolute check */
  if ((abs_trq > MAX_TORQUE) && !CURRENT_CONTROL) {
    leg->torque_error_counter++;
    if (leg->torque_error_counter >= 10) {
      turn_off_motors();
      leg->torque_error_counter = 0;
    }
  }
 
  /* Rate check */
  static int count = 0;
  if (((abs_trq - leg->previous_torque_average) / CONTROL_TIME_STEP) > MAX_TORQUE_RATE) {
    count++;
    if (count >= 10) {
      turn_off_motors();
      count = 0;
    }
  }
  leg->previous_torque_average = abs_trq;
}

#endif
