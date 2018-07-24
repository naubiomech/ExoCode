// In this file we adjust he torque reference as a function of the steps as introduced in the A_EXO_s_2_0_2


double L_stateTimerCount;
double L_flag_1 = 0;
double L_time_old_state;
double R_stateTimerCount;
double R_flag_1 = 0;
double R_time_old_state;

double R_activate_in_3_steps = 0;
double R_1st_step = 1;
double R_coef_in_3_steps = 0;
double R_start_step = 0;
double R_num_3_steps = 0;
double store_3sec_N1_RL = N1_RL;

double L_activate_in_3_steps = 0;
double L_1st_step = 1;
double L_coef_in_3_steps = 0;
double L_start_step = 0;
double L_num_3_steps = 0;

double L_coef_in_3_steps_Pctrl = 0;
double R_coef_in_3_steps_Pctrl = 0;

double L_store_N1 = 0;
double L_set_2_zero = 0;
double R_store_N1 = 0;
double R_set_2_zero = 0;

double One_time_L_set_2_zero = 1;
double One_time_R_set_2_zero = 1;


void R_ref_step_adj() {
  if (R_activate_in_3_steps == 1) {

    if (R_1st_step == 1) {
      R_coef_in_3_steps = 0;
      R_1st_step = 0;
    }

    if ((R_state == 3) && (R_state_old == 1) && (R_start_step == 0)) {
      R_start_step = 1;
      R_start_time = millis();
    }

    if (R_start_step == 1) {
      if ((R_state == 1) && (R_state_old == 3)) {
        R_start_step = 0;
        if (millis() - R_start_time >= step_time_length) { // if the transition from 3 to 1 lasted more than 0.3 sec it was a step
          R_num_3_steps += 1;

          Serial.println(R_coef_in_3_steps);
        }
      }
    }

    R_coef_in_3_steps = R_num_3_steps / 6;


    if (R_coef_in_3_steps >= 1) {
      R_coef_in_3_steps = 1;
      R_activate_in_3_steps = 0;
      R_1st_step = 1;
      R_num_3_steps = 0;
      R_start_step = 0;
    }


  }
  return;
}

void L_ref_step_adj() {

  if (L_activate_in_3_steps == 1) {

    if (L_1st_step == 1) {
      L_coef_in_3_steps = 0;
      L_coef_in_3_steps_Pctrl = 1;
      L_1st_step = 0;
    }

    if ((L_state == 3) && (L_state_old == 1) && (L_start_step == 0)) {
      L_start_time = millis();
      L_start_step = 1;
    }

    if (L_start_step == 1) {
      if ((L_state == 1) && (L_state_old == 3)) {

        L_start_step = 0;
        if (millis() - L_start_time >= step_time_length) {
          L_num_3_steps += 1;

          Serial.print("Left adj/step ");
          Serial.println(L_coef_in_3_steps);
        }
      }
    }

    L_coef_in_3_steps = L_num_3_steps / 6;

    if (L_num_3_steps >= 1) {
      L_coef_in_3_steps_Pctrl = 1;
    }
    if (L_coef_in_3_steps >= 1) {
      L_coef_in_3_steps = 1;
      L_activate_in_3_steps = 0;
      L_1st_step = 1;
      L_num_3_steps = 0;

    }


  }


  return;
}
