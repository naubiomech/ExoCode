void state_machine_RL() //For the comments on this file see the left_leg->State_Machine
{
  switch (right_leg->state)
  {
    case 1: //Swing
      if (right_leg->set_2_zero == 1) {
        right_leg->set_2_zero = 0;
        right_leg->One_time_set_2_zero = 1;
      }
      else if ((right_leg->p_steps->curr_voltage > right_leg->fsr_percent_thresh_Toe * right_leg->fsr_Combined_peak_ref)) //&& (fsr(fsr_sense_Long) < (fsr_thresh_long * fsr_cal_Long)))
      {
        right_leg->state_count_13++;
        if (right_leg->state_count_13 >= state_counter_th)
        {
          right_leg->sigm_done = true;
          right_leg->Old_PID_Setpoint = right_leg->PID_Setpoint;
          //          right_leg->New_PID_Setpoint = right_leg->Setpoint_Ankle * right_leg->coef_in_3_steps;
          if (right_leg->Previous_Setpoint_Ankle <= right_leg->Setpoint_Ankle) {

            right_leg->New_PID_Setpoint = right_leg->Previous_Setpoint_Ankle + (right_leg->Setpoint_Ankle - right_leg->Previous_Setpoint_Ankle) * right_leg->coef_in_3_steps;

          } else {

            right_leg->New_PID_Setpoint = right_leg->Previous_Setpoint_Ankle - (right_leg->Previous_Setpoint_Ankle - right_leg->Setpoint_Ankle) * right_leg->coef_in_3_steps;

          }

          right_leg->state_old = right_leg->state;
          right_leg->state = 3;
          right_leg->state_count_13 = 0;
          right_leg->state_count_31 = 0;
        }
      }
      break;
    case 3: //Late Stance

      if ((right_leg->set_2_zero == 1) && (right_leg->One_time_set_2_zero)) {
        right_leg->sigm_done = true;
        right_leg->Old_PID_Setpoint = right_leg->PID_Setpoint;
        right_leg->state_old = right_leg->state;
        right_leg->New_PID_Setpoint = 0;
        right_leg->One_time_set_2_zero = 0;
        right_leg->Previous_Setpoint_Ankle = 0;
        right_leg->Setpoint_Ankle_Pctrl = 0;

      }

      if ((right_leg->p_steps->curr_voltage < (right_leg->fsr_percent_thresh_Toe * right_leg->fsr_Combined_peak_ref)))
      {
        right_leg->state_count_31++;
        if (right_leg->state_count_31 >= state_counter_th)
        {
          right_leg->sigm_done = true;
          right_leg->Old_PID_Setpoint = right_leg->PID_Setpoint;
          right_leg->state_old = right_leg->state;
          right_leg->New_PID_Setpoint = 0 * right_leg->coef_in_3_steps;
          right_leg->state = 1;
          right_leg->state_count_31 = 0;
          right_leg->state_count_13 = 0;
        }
      }
      break;
  }

  R_ref_step_adj();


  if ((Trq_time_volt == 2 || Trq_time_volt == 3) && right_leg->state == 3) {
    right_leg->PID_Setpoint = right_leg->Setpoint_Ankle_Pctrl;
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      right_leg->PID_Setpoint = right_leg->New_PID_Setpoint;
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve_RL();
    }

  }

}

