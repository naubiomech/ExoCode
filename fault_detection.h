#include "ema_filter.h"
#include "motor_utils.h"


#ifndef FAULT_DETECTION
#define FAULT_DETECTION

/* The fault detection is currently performed in tracking_check() and torque_check(). If the exo is tracking poorly,
 * then the torque sensor may be bad. The tunable parameters are included as defines below. The 
 * tracking_check() function performs the error checking. By applying an EMA filter on the setpoint
 * the torque response may be approximated. The error between this approximated value and the actual
 * torque response are computed, and passed through an EMA filter. The averaged error is compared
 * to the tracking_thresh define and, if greater, an error is thrown and the motors are turned off. In
 * the event that a false positive is thrown the operator may turn the motors off and on with the 
 * pause/play functionality on the app. 
 */

//Tracking error in N*m
#define TRACKING_THRESH       10.0
//Estimated torque alpha
#define EST_TRQ_ALPHA         0.1
//Error alpha
#define ERROR_ALPHA           0.005
//Tracking error rate in (N*m)/s
#define TRACKING_RATE_THRESH  2 * TRACKING_THRESH / CONTROL_TIME_STEP
//Max torque setpoint
#define MAX_TORQUE            35
//Max torque rate
#define MAX_TORQUE_RATE       20 * MAX_TORQUE / CONTROL_TIME_STEP
//Max PID saturate time in seconds
#define MAX_SAT_TIME          1
//SAT TIME COUNT
#define SAT_COUNT             MAX_SAT_TIME / CONTROL_TIME_STEP

//Protocols
inline void check_for_torque_faults(Leg* leg);
inline void tracking_check(Leg* leg);
inline void torque_check(Leg* leg);
inline void pid_check(Leg* leg);


inline void detect_faults() {
  check_for_torque_faults(right_leg);
  check_for_torque_faults(left_leg);
}

inline void check_for_torque_faults(Leg* leg) {
  tracking_check(leg);
  torque_check(leg);
  pid_check(leg);
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
    filtered_error_R *= sign;
    if ((filtered_error_R > TRACKING_THRESH || (sign * track_error_rate) > TRACKING_RATE_THRESH) && stream) {
      Serial.println("R_TRACK");
      change_motor_state(false);
    }
    track_error_R = filtered_error_R; 
  } 
  else {
    filtered_sp_L = ema_with_context(filtered_sp_L, left_leg->PID_Setpoint, EST_TRQ_ALPHA);
    double current_track_error = abs(filtered_sp_L - leg->Average_Trq);
    filtered_error_L = abs(ema_with_context(filtered_error_L, current_track_error, ERROR_ALPHA));
    double track_error_rate = abs((filtered_error_L - track_error_L) / CONTROL_TIME_STEP);
    filtered_error_L *= sign;
    if ((filtered_error_L > TRACKING_THRESH || (sign * track_error_rate) > TRACKING_RATE_THRESH) && stream) {
      Serial.println("L_TRACK");
      change_motor_state(false);
    }
    track_error_L = filtered_error_L;
  }
}

inline void torque_check(Leg* leg) {
  double abs_trq = abs(leg->Average_Trq);
  /* Absolute check */
  if ((abs_trq > MAX_TORQUE) && !CURRENT_CONTROL) {
    leg->torque_error_counter++;
    if (leg->torque_error_counter >= 10) {
      change_motor_state(false);
      Serial.println("TRQ_MAX");
      leg->torque_error_counter = 0;
    }
  }
 
  /* Rate check */
  static int count = 0;
  if (((abs_trq - leg->previous_torque_average) / CONTROL_TIME_STEP) > MAX_TORQUE_RATE) {
    count++;
    if (count >= 10) {
      change_motor_state(false);
      Serial.println("TRQ_RATE");
      count = 0;
    }
  }
  leg->previous_torque_average = abs_trq;
}

inline void pid_check(Leg* leg) {
  static uint32_t r_count = 0;
  static uint32_t l_count = 0;
  
  if (leg == right_leg) {
    if ((abs(leg->Output)) >= 1400) {
      r_count++;
      if (count >= SAT_COUNT) {
        change_motor_state(false);
        Serial.println("R PID ERR");
      }
    } else {
      r_count = 0;
    }
  } else if (leg == left_leg) {
    if ((abs(leg->Output)) >= 1400) {
      l_count++;
      if (count >= SAT_COUNT) {
        change_motor_state(false);
        Serial.println("L PID ERR");
      }
    } else {
      l_count = 0;
    }
  }
}

#endif
