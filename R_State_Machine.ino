void state_machine_RL() //For the comments on this file see the L_State_Machine
{
  switch (R_state)
  {
    case 1: //Swing
      if (R_set_2_zero == 1) {
        R_set_2_zero = 0;
        One_time_R_set_2_zero = 1;
      }
      else if ((R_p_steps->curr_voltage > fsr_percent_thresh_Right_Toe * fsr_Right_Combined_peak_ref)) //&& (fsr(fsr_sense_Long) < (fsr_thresh_long * fsr_cal_Long)))
      {
        state_count_RL_13++;
        if (state_count_RL_13 >= state_counter_th)
        {
          sigm_done_RL = true;
          Old_PID_Setpoint_RL = PID_Setpoint_RL;
          //          New_PID_Setpoint_RL = Setpoint_Ankle_RL * R_coef_in_3_steps;
          if (Previous_Setpoint_Ankle_RL <= Setpoint_Ankle_RL) {

            New_PID_Setpoint_RL = Previous_Setpoint_Ankle_RL + (Setpoint_Ankle_RL - Previous_Setpoint_Ankle_RL) * R_coef_in_3_steps;

          } else {

            New_PID_Setpoint_RL = Previous_Setpoint_Ankle_RL - (Previous_Setpoint_Ankle_RL - Setpoint_Ankle_RL) * R_coef_in_3_steps;

          }

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
        Previous_Setpoint_Ankle_RL = 0;
        Setpoint_Ankle_RL_Pctrl = 0;

      }

      if ((R_p_steps->curr_voltage < (fsr_percent_thresh_Right_Toe * fsr_Right_Combined_peak_ref)))
      {
        state_count_RL_31++;
        if (state_count_RL_31 >= state_counter_th)
        {
          sigm_done_RL = true;
          Old_PID_Setpoint_RL = PID_Setpoint_RL;
          R_state_old = R_state;
          New_PID_Setpoint_RL = 0 * R_coef_in_3_steps;
          R_state = 1;
          state_count_RL_31 = 0;
          state_count_RL_13 = 0;
        }
      }
      break;
  }

  R_ref_step_adj();


  if ((Trq_time_volt == 2 || Trq_time_volt ==3) && R_state == 3) {
PID_Setpoint_RL = Setpoint_Ankle_RL_Pctrl;
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      PID_Setpoint_RL = New_PID_Setpoint_RL;
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve_RL();
    }
    
  }

}

