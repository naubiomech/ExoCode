// In this file we adjust he torque reference as a function of the steps as introduced in the A_EXO_s_2_0_2
#ifndef REFERENCE_ADJ_HEADER
#define REFERENCE_ADJ_HEADER

#include "Leg.h"

void ref_step_adj(Leg* leg);

void R_ref_step_adj() {
  ref_step_adj(right_leg);
}

void L_ref_step_adj() {
  ref_step_adj(left_leg);
}
#endif
