void state_machine_RL()
{
  switch (R_state)
  {
    case 1: //Swing
      if ((fsr(fsr_sense_Right_Toe) > fsr_percent_thresh_Right_Toe * fsr_Right_Toe_thresh)) //&& (fsr(fsr_sense_Long) < (fsr_thresh_long * fsr_cal_Long)))
      {
        state_count_RL_13++;
        if (state_count_RL_13 >= 4)
        {
          sigm_done_RL = true;
          Old_PID_Setpoint_RL = PID_Setpoint_RL;
          New_PID_Setpoint_RL = Setpoint_Ankle_RL;
          R_state_old = R_state;
          R_state = 3;
          state_count_RL_13 = 0;
        }
      }
      break;
    case 3: //Late Stance
      if ((fsr(fsr_sense_Right_Toe) < (fsr_percent_thresh_Right_Toe * fsr_Right_Toe_thresh)))
      {
        state_count_RL_31++;
        if (state_count_RL_31 >= 4)
        {
          sigm_done_RL = true;
          Old_PID_Setpoint_RL = PID_Setpoint_RL;
          R_state_old = R_state;
          New_PID_Setpoint_RL = 0;
          //            PID_Setpoint = 0;//-3; //Dorsiflexion for MAriah in Swing, otherwise should be 0
          R_state = 1;
          state_count_RL_31 = 0;
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
  PID_Curve_RL();
}

void state_machine_LL()
{
  switch (L_state)
  {
    case 1: //Swing
      //        Serial.println("L_State 1");
      if ((fsr(fsr_sense_Left_Toe) > fsr_percent_thresh_Left_Toe * fsr_Left_Toe_thresh)) //&& (fsr(fsr_sense_Long) < (fsr_thresh_long * fsr_cal_Long)))
      {
//        Serial.println("Inside case 1 New L state is");
//        Serial.println(L_state);
        state_count_LL_13++;
//        Serial.println(state_count_LL_13);
        if (state_count_LL_13 >= 4)
        {
          sigm_done_LL = true;
          Old_PID_Setpoint_LL = PID_Setpoint_LL;
          New_PID_Setpoint_LL = Setpoint_Ankle_LL;
          L_state_old = L_state;
          L_state = 3;
          state_count_LL_13 = 0;
        }
      }
      break;
    case 3: //Late Stance
//      Serial.println("L_State 3");
      if ((fsr(fsr_sense_Left_Toe) < (fsr_percent_thresh_Left_Toe * fsr_Left_Toe_thresh)))
      {
        state_count_LL_31++;
        if (state_count_LL_31 >= 4)
        {
          sigm_done_LL = true;
          Old_PID_Setpoint_LL = PID_Setpoint_LL;
          L_state_old = L_state;
          New_PID_Setpoint_LL = 0;
          //            PID_Setpoint = 0;//-3; //Dorsiflexion for MAriah in Swing, otherwise should be 0
          L_state = 1;
          state_count_LL_31 = 0;
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
  PID_Curve_LL();
}

