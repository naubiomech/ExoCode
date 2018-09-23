#include "Steady_State.h"
// If the participant keeps the state more than 3 seconds, the torque reference is set to 0

void set_to_zero_if_leg_in_steady_state(Leg* leg){
  if (leg->flag_1 == 0) {
    leg->flag_1 = 1;
    leg->time_old_state = leg->state;
  }
  if (leg->state != leg->time_old_state) {
    leg->flag_1 = 0;
    leg->stateTimerCount = 0;
  } else {
    if (leg->stateTimerCount >= 5 / 0.002) {
      set_to_zero(leg);
    } else {
      leg->stateTimerCount++;
      if (leg->store_N1) {
        leg->set_2_zero = 0;
        leg->store_N1 = 0;
      }
    }
  }
}

void set_to_zero(Leg* leg){
    if (leg->store_N1 == 0) {
      Serial.println("Steady state, setting to 0Nm , Change N1");
      leg->set_2_zero = 1;
      leg->store_N1 = 1;
      leg->activate_in_3_steps = 1;
      leg->num_3_steps = 0;
      leg->first_step = 1;
      leg->start_step = 0;
    }
}

void set_2_zero_if_steady_state() {
  set_to_zero_if_leg_in_steady_state(right_leg);
  set_to_zero_if_leg_in_steady_state(left_leg);
}
