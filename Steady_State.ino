#include "Steady_State.h"
// If the participant keeps the state more than 3 seconds, the torque reference is set to 0


void set_2_zero_if_steady_state() {

  if (left_leg->flag_1 == 0) {
    left_leg->flag_1 = 1;
    left_leg->time_old_state = left_leg->state;
  }
  if (left_leg->state != left_leg->time_old_state) {
    left_leg->flag_1 = 0;
    left_leg->stateTimerCount = 0;
  } else {
    if (left_leg->stateTimerCount >= 5 / 0.002) {
      if (left_leg->store_N1 == 0) {
        Serial.println("Steady state, setting to 0Nm , Change N1");
        left_leg->set_2_zero = 1;
        left_leg->store_N1 = 1;
        left_leg->activate_in_3_steps = 1;
        left_leg->num_3_steps = 0;
        left_leg->first_step = 1;
        left_leg->start_step = 0;
      }

    } else {
      left_leg->stateTimerCount++;
      if (left_leg->store_N1) {
        left_leg->set_2_zero = 0;
        left_leg->store_N1 = 0;
      }
    }
  }

  if (right_leg->flag_1 == 0) {
    right_leg->flag_1 = 1;
    right_leg->time_old_state = right_leg->state;
  }
  if (right_leg->state != right_leg->time_old_state) {
    right_leg->flag_1 = 0;
    right_leg->stateTimerCount = 0;
  } else {
    if (right_leg->stateTimerCount >= 5 / 0.002) {
      if (right_leg->store_N1 == 0) {
        Serial.println("Change N1");
        right_leg->set_2_zero = 1;
        right_leg->store_N1 = 1;
        right_leg->activate_in_3_steps = 1;
        right_leg->num_3_steps = 0;

        right_leg->first_step = 1;
        right_leg->start_step = 0;
        right_leg->coef_in_3_steps_Pctrl = 0;
      }
    } else {
      right_leg->stateTimerCount++;
      if (right_leg->store_N1) {
        right_leg->set_2_zero = 0;
        right_leg->store_N1 = 0;
      }
    }
  }


  return;
}
