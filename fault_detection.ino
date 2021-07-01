#include "ema_filter.h"

//Tracking error in N*m
#define TRACKING_THRESH       5.0
//Tracking error rate in (N*m)/s
#define TRACKING_RATE_THRESH  TRACKING_THRESH / CONTROL_TIME_STEP

void detect_faults() {
  check_for_torque_faults(right_leg);
  check_for_torque_faults(left_leg);
}

void check_for_torque_faults(Leg* leg) {
  tracking_check(leg);
}

int tracking_check(Leg* leg) {
  //Compare smoothed tracking (LPF SP) with measured torque
  static double filtered_sp_L = 0;
  static double filtered_sp_R = 0;
  static double track_error_L = 0;
  static double track_error_R = 0;
  static double filtered_error_L = 0;
  static double filtered_error_R = 0;

  if (leg == right_leg) {
    filtered_sp_R = ema_with_context(filtered_sp_R, right_leg->PID_Setpoint, MED_ALPHA);
    double current_track_error = abs(filtered_sp_R - leg->Average_Trq);
    filtered_error_R = abs(ema_with_context(filtered_error_R, current_track_error, HIGH_ALPHA));
    double track_error_rate = abs((current_track_error - track_error_R) / CONTROL_TIME_STEP);
    if (filtered_error_R > TRACKING_THRESH || track_error_rate > TRACKING_RATE_THRESH) {
      digitalWrite(onoff, LOW);                                         //The GUI user is ready to end the trial, so motor is disabled
      stream = 0;                                                    //and the torque data is no longer allowed to be streamed.
      stepper->steps = 0;                                            // Reset step count for next trial
      reset_cal_on_end_trial(); //Currently commented out but acts to reset all torque, BL, and FSR parameters
    }
    track_error_R = filtered_error_R;
  } 
  else {
    filtered_sp_L = ema_with_context(filtered_sp_L, left_leg->PID_Setpoint, MED_ALPHA);
    double current_track_error = abs(filtered_sp_L - leg->Average_Trq);
    filtered_error_L = abs(ema_with_context(filtered_error_L, current_track_error, HIGH_ALPHA));
    double track_error_rate = abs((filtered_error_L - track_error_L) / CONTROL_TIME_STEP);
    if (filtered_error_L > TRACKING_THRESH || track_error_rate > TRACKING_RATE_THRESH) {
      digitalWrite(onoff, LOW);                                         //The GUI user is ready to end the trial, so motor is disabled
      stream = 0;                                                    //and the torque data is no longer allowed to be streamed.
      stepper->steps = 0;                                            // Reset step count for next trial
      reset_cal_on_end_trial(); //Currently commented out but acts to reset all torque, BL, and FSR parameters
    }
    track_error_L = filtered_error_L;
  }
}
