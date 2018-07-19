// If the participant keeps the state more than 3 seconds, the torque reference is set to 0


void set_2_zero_if_steady_state() {
  if (L_flag_1 == 0) {
    L_flag_1 = 1;
    L_time_old_state = L_state;
  }
  if (L_state != L_time_old_state) {
    L_flag_1 = 0;
    L_stateTimerCount = 0;
  } else {
    if (L_stateTimerCount >= 2.5 / 0.002) {
      //        Serial.println("Too Long");
      if (L_store_N1 == 0) {
        Serial.println("Steady state, setting to 0Nm , Change N1");
        L_set_2_zero = 1;
        L_store_N1 = 1;
        L_activate_in_3_steps = 1;
        L_num_3_steps = 0;
        L_1st_step = 1;
        L_start_step = 0;
      }

    } else {
      L_stateTimerCount++;
      if (L_store_N1) {
        L_set_2_zero = 0;
        L_store_N1 = 0;
      }
    }
  }

  if (R_flag_1 == 0) {
    R_flag_1 = 1;
    R_time_old_state = R_state;
  }
  if (R_state != R_time_old_state) {
    R_flag_1 = 0;
    R_stateTimerCount = 0;
  } else {
    if (R_stateTimerCount >= 2.5 / 0.002) {
      if (R_store_N1 == 0) {
        Serial.println("Change N1");
        R_set_2_zero = 1;
        R_store_N1 = 1;
        R_activate_in_3_steps = 1;
        R_num_3_steps = 0;
        R_1st_step = 1;
        R_start_step = 0;
      }
    } else {
      R_stateTimerCount++;
      if (R_store_N1) {
        R_set_2_zero = 0;
        R_store_N1 = 0;
      }
    }
  }


  return;
}
