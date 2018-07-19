void state_machine_RL()
{
  switch (R_state)
  {
    case 1: //Swing
      if (R_set_2_zero == 1) {
        R_set_2_zero = 0;
        One_time_R_set_2_zero = 1;
      }
      else if ((fsr(fsr_sense_Right_Toe) > fsr_percent_thresh_Right_Toe * fsr_Right_Toe_thresh)) //&& (fsr(fsr_sense_Long) < (fsr_thresh_long * fsr_cal_Long)))
      {
        state_count_RL_13++;
        if (state_count_RL_13 >= state_counter_th)
        {
          sigm_done_RL = true;
          Old_PID_Setpoint_RL = PID_Setpoint_RL;
          New_PID_Setpoint_RL = Setpoint_Ankle_RL * R_coef_in_3_steps;
          R_state_old = R_state;
          R_state = 3;
          state_count_RL_13 = 0;
          state_count_RL_31 = 0;
        }
      }
      break;
    case 3: //Late Stance

      if ((R_set_2_zero == 1) && (One_time_R_set_2_zero)) {
        sigm_done_RL = true;
        Old_PID_Setpoint_RL = PID_Setpoint_RL;
        R_state_old = R_state;
        New_PID_Setpoint_RL = 0;
        One_time_R_set_2_zero = 0;

      }

      if ((fsr(fsr_sense_Right_Toe) < (fsr_percent_thresh_Right_Toe * fsr_Right_Toe_thresh)))
      {
        state_count_RL_31++;
        if (state_count_RL_31 >= state_counter_th)
        {
          sigm_done_RL = true;
          Old_PID_Setpoint_RL = PID_Setpoint_RL;
          R_state_old = R_state;
          New_PID_Setpoint_RL = 0 * R_coef_in_3_steps;
          //            PID_Setpoint = 0;//-3; //Dorsiflexion for MAriah in Swing, otherwise should be 0
          R_state = 1;
          state_count_RL_31 = 0;
          state_count_RL_13 = 0;
        }
      }
      //        case 1: //Swing
      //          if ((fsr(fsr_sense_Right_Heel) > fsr_percent_thresh_Right_Heel * fsr_Right_Heel_thresh)) //&& (fsr(fsr_sense_Long) < (fsr_thresh_long * fsr_cal_Long)))
      //          {
      //            state_count_RL_13++;
      //            if (state_count_RL_13 == 4)
      //            {
      //              sigm_done_RL = true;
      //              Old_PID_Setpoint_RL = PID_Setpoint_RL;
      //              New_PID_Setpoint_RL = Setpoint_Ankle_RL;
      //              R_state_old = R_state;
      //              R_state = 3;
      //              state_count_RL_13 = 0;
      //            }
      //          }
      //          break;
      //        case 3: //Late Stance
      //          if ((fsr(fsr_sense_Right_Toe) < (fsr_percent_thresh_Right_Toe * fsr_Right_Toe_thresh)))
      //          {
      //            state_count_RL_31++;
      //            if (state_count_RL_31 == 4)
      //            {
      //              sigm_done_RL = true;
      //              Old_PID_Setpoint_RL = PID_Setpoint_RL;
      //              R_state_old = R_state;
      //              New_PID_Setpoint_RL = 0;
      //              //            PID_Setpoint = 0;//-3; //Dorsiflexion for MAriah in Swing, otherwise should be 0
      //              R_state = 1;
      //              state_count_RL_31 = 0;
      //            }
      //          }
      break;
  }


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

  PID_Curve_RL();

}




void state_machine_LL()
{
  switch (L_state)
  {
    case 1: //Swing
      //        Serial.println("L_State 1");
      if (L_set_2_zero == 1) {
        L_set_2_zero = 0;
        One_time_L_set_2_zero = 1;
      }
      else if ((fsr(fsr_sense_Left_Toe) > fsr_percent_thresh_Left_Toe * fsr_Left_Toe_thresh)) //&& (fsr(fsr_sense_Long) < (fsr_thresh_long * fsr_cal_Long)))
      {
        //        Serial.println("Inside case 1 New L state is");
        //        Serial.println(L_state);
        state_count_LL_13++;
        //        Serial.println(state_count_LL_13);
        if (state_count_LL_13 >= state_counter_th)
        {
          sigm_done_LL = true;
          Old_PID_Setpoint_LL = PID_Setpoint_LL;
          New_PID_Setpoint_LL = Setpoint_Ankle_LL * L_coef_in_3_steps;
          L_state_old = L_state;
          L_state = 3;
          state_count_LL_13 = 0;
          state_count_LL_31 = 0;
        }
      }

      break;
    case 3: //Late Stance
      if ((L_set_2_zero == 1) && (One_time_L_set_2_zero)) {
        sigm_done_LL = true;
        Old_PID_Setpoint_LL = PID_Setpoint_LL;
        L_state_old = L_state;
        New_PID_Setpoint_LL = 0;
        One_time_L_set_2_zero = 0;

      }

      //      Serial.println("L_State 3");
      if ((fsr(fsr_sense_Left_Toe) < (fsr_percent_thresh_Left_Toe * fsr_Left_Toe_thresh)))
      {
        state_count_LL_31++;
        if (state_count_LL_31 >= state_counter_th)
        {
          sigm_done_LL = true;
          Old_PID_Setpoint_LL = PID_Setpoint_LL;
          L_state_old = L_state;
          New_PID_Setpoint_LL = 0 * L_coef_in_3_steps;
          //            PID_Setpoint = 0;//-3; //Dorsiflexion for MAriah in Swing, otherwise should be 0
          L_state = 1;
          state_count_LL_31 = 0;
          state_count_LL_13 = 0;
        }
      }


      //        case 1: //Swing
      //          if ((fsr(fsr_sense_Left_Heel) > fsr_percent_thresh_Left_Heel * fsr_Left_Heel_thresh)) //&& (fsr(fsr_sense_Long) < (fsr_thresh_long * fsr_cal_Long)))
      //          {
      //            state_count_LL_13++;
      //            if (state_count_LL_13 == 4)
      //            {
      //              sigm_done_LL = true;
      //              Old_PID_Setpoint_LL = PID_Setpoint_LL;
      //              New_PID_Setpoint_LL = Setpoint_Ankle_LL;
      //              L_state_old = L_state;
      //              L_state = 3;
      //              state_count_LL_13 = 0;
      //            }
      //          }
      //          break;
      //        case 3: //Late Stance
      //          if ((fsr(fsr_sense_Left_Toe) < (fsr_percent_thresh_Left_Toe * fsr_Left_Toe_thresh)))
      //          {
      //            state_count_LL_31++;
      //            if (state_count_LL_31 == 4)
      //            {
      //              sigm_done_LL = true;
      //              Old_PID_Setpoint_LL = PID_Setpoint_LL;
      //              L_state_old = L_state;
      //              New_PID_Setpoint_LL = 0;
      //              //            PID_Setpoint = 0;//-3; //Dorsiflexion for MAriah in Swing, otherwise should be 0
      //              L_state = 1;
      //              state_count_LL_31 = 0;
      //            }
      //          }
      break;
  }

  if (L_activate_in_3_steps == 1) {

    if (L_1st_step == 1) {
      L_coef_in_3_steps = 0;
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
          Serial.println(L_coef_in_3_steps);
        }
      }
    }

    L_coef_in_3_steps = L_num_3_steps / 6;

    if (L_coef_in_3_steps >= 1) {
      L_coef_in_3_steps = 1;
      L_activate_in_3_steps = 0;
      L_1st_step = 1;
      L_num_3_steps = 0;
    }


  }


  PID_Curve_LL();
}

