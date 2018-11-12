#include "Reference_ADJ.h"

void ref_step_adj(Leg* leg) {
  if (leg->activate_in_3_steps == 1) {

    if (leg->first_step == 1) {
      leg->coef_in_3_steps = 0;
      leg->first_step = 0;
    }

    if ((leg->state == 3) && (leg->state_old == 1) && (leg->start_step == 0)) {
      leg->start_step = 1;
      leg->start_time = millis();
    }

    if (leg->start_step == 1) {
      if ((leg->state == 1) && (leg->state_old == 3)) {
        leg->start_step = 0;
        if (millis() - leg->start_time >= step_time_length) { // if the transition from 3 to 1 lasted more than 0.3 sec it was a step
          leg->num_3_steps += 1;

          Serial.println(leg->coef_in_3_steps);
        }
      }
    }

    leg->coef_in_3_steps = leg->num_3_steps / 6;


    if (leg->coef_in_3_steps >= 1) {
      leg->coef_in_3_steps = 1;
      leg->activate_in_3_steps = 0;
      leg->first_step = 1;
      leg->num_3_steps = 0;
      leg->start_step = 0;
    }


  }
  return;
}
