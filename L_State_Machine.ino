void state_machine_LL()
{
  switch (left_leg->state)
  {
    case 1: //Swing
      // This flag enables the "set to zero" procedure for the left ankle.
      // When you're for at least 3 seconds in the same state, the torque reference is set to zero
      if (left_leg->set_2_zero == 1) {
        left_leg->set_2_zero = 0;
        left_leg->One_time_set_2_zero = 1;
      }
      else if ((left_leg->p_steps->curr_voltage > left_leg->fsr_percent_thresh_Toe * left_leg->fsr_Combined_peak_ref)) //&& (fsr(fsr_sense_Long) < (fsr_thresh_long * fsr_cal_Long)))
      {
        left_leg->state_count_13++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (left_leg->state_count_13 >= state_counter_th)
        {
          left_leg->sigm_done = true;
          left_leg->Old_PID_Setpoint = left_leg->PID_Setpoint;
          if (left_leg->Previous_Setpoint_Ankle <= left_leg->Setpoint_Ankle) {

            left_leg->New_PID_Setpoint = left_leg->Previous_Setpoint_Ankle + (left_leg->Setpoint_Ankle - left_leg->Previous_Setpoint_Ankle) * left_leg->coef_in_3_steps;

          } else {

            left_leg->New_PID_Setpoint = left_leg->Previous_Setpoint_Ankle - (left_leg->Previous_Setpoint_Ankle - left_leg->Setpoint_Ankle) * left_leg->coef_in_3_steps;

          }

          left_leg->state_old = left_leg->state;
          left_leg->state = 3;
          left_leg->state_count_13 = 0;
          left_leg->state_count_31 = 0;
        }
      }

      break;
    case 3: //Late Stance
      if ((left_leg->set_2_zero == 1) && (left_leg->One_time_set_2_zero)) {
        left_leg->sigm_done = true;
        left_leg->Old_PID_Setpoint = left_leg->PID_Setpoint;
        left_leg->state_old = left_leg->state;
        left_leg->New_PID_Setpoint = 0;
        left_leg->One_time_set_2_zero = 0;
        left_leg->Previous_Setpoint_Ankle = 0;
        left_leg->PID_Setpoint = 0;
        left_leg->Setpoint_Ankle_Pctrl = 0;
      }

      if ((left_leg->p_steps->curr_voltage < (left_leg->fsr_percent_thresh_Toe * left_leg->fsr_Combined_peak_ref)))
      {
        left_leg->state_count_31++;
        if (left_leg->state_count_31 >= state_counter_th)
        {
          left_leg->sigm_done = true;
          left_leg->Old_PID_Setpoint = left_leg->PID_Setpoint;
          left_leg->state_old = left_leg->state;
          left_leg->New_PID_Setpoint = 0 * left_leg->coef_in_3_steps;
          left_leg->state = 1;
          left_leg->state_count_31 = 0;
          left_leg->state_count_13 = 0;
        }
      }
  }
  // Adjust the torque reference as a function of the step
  L_ref_step_adj();

  if ((Trq_time_volt == 2 || Trq_time_volt == 3) && left_leg->state == 3) {
    left_leg->PID_Setpoint = left_leg->Setpoint_Ankle_Pctrl;
    Serial.println("After switch case : ");
    Serial.println(left_leg->coef_in_3_steps_Pctrl);
    Serial.println(left_leg->Setpoint_Ankle_Pctrl);
    Serial.println(left_leg->PID_Setpoint);
    Serial.println();
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      left_leg->PID_Setpoint = left_leg->New_PID_Setpoint;
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve_LL();
    }

  }

}
