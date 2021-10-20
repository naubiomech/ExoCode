
//--------------------NEW FUNCTIONS-----------------
// Before state 3 in case of balance control was activated just in case of not negligible value
// of the force on the toe i.e. if you just walk on the heel you never feel the balance.
// Right now the state 3 is activate also in case of heel contact and the state 1 is activated
// in case of no contact of both the sensors
void state_machine(Leg* leg)
{
//  if (FLAG_ONE_TOE_SENSOR) {
//    State_Machine_One_Toe_Sensor(leg);
//  }
//
//  else if (FLAG_BALANCE) {
//    State_Machine_Heel_Toe_Sensors_Balance(leg);
//  } else {
//    if (FLAG_BIOFEEDBACK) {
//      State_Machine_Heel_Toe_Sensors_BioFeedback(leg);
//    } else {
//      State_Machine_Heel_Toe_Sensors(leg);
//    }
//  }

  if(Control_Mode != 100){
    if (Line == 1)
      State_Machine_Heel_Toe_Sensors_Hip_3State(leg);                                                 // The statemachine desiened for the line connection between extension and flextion during swing. 1) mid-late swing, 3) early-mid stance, 5) Stance to swing connection
    else
      State_Machine_Heel_Toe_Sensors_Hip(leg);
      } else{
        State_Machine_BangBang_Hip(leg);                                                              // Bang Banghip controller contains 5 gait phases: 1) mid-late swing, 2) early stance, 3) mid-stance,     4) late-stance,           5) early swing 
        }                                                                                             // Setpoint is also assined inside the statemachine: 1) zero,         2) zero,         3) Setpoint_Ankle, 4) Dorsi_Setpoint_Ankle/2 5)Dorsi_Setpoint_Ankle
}




//-----------------------------------------------------------------------------------------------------
void State_Machine_One_Toe_Sensor(Leg * leg) {
  switch (leg->state)
  {
    case 1: //Swing
      // This flag enables the "set to zero" procedure for the left ankle.
      // When you're for at least 3 seconds in the same state, the torque reference is set to zero
      if (leg->set_2_zero == 1) {
        leg->set_2_zero = 0;
        leg->One_time_set_2_zero = 1;
      }
      else if ((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)) //If the overall FSR reading is greater than the threshold we need to be in state 3
      {
        leg->state_count_13++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_13 >= state_counter_th)
        {
          if (Control_Mode == 100) { //Increment set point for bang-bang GO - 5/19/19
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            if (leg->Previous_Setpoint_Ankle <= leg->Setpoint_Ankle) {

              leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps;

            } else {

              leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle - (leg->Previous_Setpoint_Ankle - leg->Setpoint_Ankle) * leg->coef_in_3_steps;

            }

            if (Flag_HLO && (leg->Previous_T_Opt <= leg->T_Opt)) {

              leg->T_Opt_Setpoint = leg->Previous_T_Opt + (leg->T_Opt - leg->Previous_T_Opt) * leg->coef_in_3_steps;

            } else if (Flag_HLO && (leg->Previous_T_Opt > leg->T_Opt)) {

              leg->T_Opt_Setpoint = leg->Previous_T_Opt - (leg->Previous_T_Opt - leg->T_Opt) * leg->coef_in_3_steps;
            }
          }

          else if (Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4 || Control_Mode == 6) { //Increment the max set point for proportional control GO - 5/19/19
            if (abs(leg->Previous_Setpoint_Ankle_Pctrl) <= abs(leg->Setpoint_Ankle)) {
              leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle_Pctrl) * leg->coef_in_3_steps; //Increment when the new setpoint is higher than the old
            } else {
              leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl - (leg->Previous_Setpoint_Ankle_Pctrl - leg->Setpoint_Ankle) * leg->coef_in_3_steps; //Decrement when the new setpoint is lower than the old
            }
            if (leg->p_steps->Setpoint == 0) {
              leg->Previous_Setpoint_Ankle_Pctrl = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
            }
          }

          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
          leg->state_count_32 = 0;
          leg->state_count_23 = 0;
        }
      }
      else if ((leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref && leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)) { //Heel active but not toe
        if (Control_Mode != 100) {
          leg->state_count_12++;
          if (leg->state_count_12 >= state_counter_th) {
            leg->state_old = leg->state;
            leg->state = 2;
            leg->state_count_23 = 0;
            leg->state_count_32 = 0;
            leg->state_count_21 = 0;
            leg->state_count_12 = 0;
          }
        }
      }

      break;

    case 2: //Early Stance //GO 5/19/19

      if (leg->state_old == 1 && leg->state == 2) { //TH 5/23/19
        if (leg->allow_inc_flag == true) {
          leg->step_count++;
          leg->allow_inc_flag = false;
        }
      }

      if (leg->state_old == 1 && leg->step_count > 9) {  //TH 5/23/19
        leg->TM_data = 1;
        leg->step_count = 0;
      }

      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true;
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->PID_Setpoint = 0;
        leg->Setpoint_Ankle_Pctrl = 0;
        leg->Previous_Setpoint_Ankle_Pctrl = 0;
      }
      else if ((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)) {
        leg->state_count_23++;
        if (leg->state_count_23 >= state_counter_th) {

          if (Control_Mode == 100) { //Increment set point for bang-bang GO - 5/19/19
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            if (leg->Previous_Setpoint_Ankle <= leg->Setpoint_Ankle) {

              leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps;

            } else {

              leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle - (leg->Previous_Setpoint_Ankle - leg->Setpoint_Ankle) * leg->coef_in_3_steps;

            }

            if (Flag_HLO && (leg->Previous_T_Opt <= leg->T_Opt)) {

              leg->T_Opt_Setpoint = leg->Previous_T_Opt + (leg->T_Opt - leg->Previous_T_Opt) * leg->coef_in_3_steps;

            } else if (Flag_HLO && (leg->Previous_T_Opt > leg->T_Opt)) {

              leg->T_Opt_Setpoint = leg->Previous_T_Opt - (leg->Previous_T_Opt - leg->T_Opt) * leg->coef_in_3_steps;
            }
          }

          else if (Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4 || Control_Mode == 6) { //Increment the max set point for proportional control GO - 5/19/19
            if (abs(leg->Previous_Setpoint_Ankle_Pctrl) <= abs(leg->Setpoint_Ankle)) {
              leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle_Pctrl) * leg->coef_in_3_steps; //Increment when the new setpoint is higher than the old
            } else {
              leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl - (leg->Previous_Setpoint_Ankle_Pctrl - leg->Setpoint_Ankle) * leg->coef_in_3_steps; //Decrement when the new setpoint is lower than the old
            }
            if (leg->p_steps->Setpoint == 0) {
              leg->Previous_Setpoint_Ankle_Pctrl = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
            }
          }

          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_count_23 = 0;
          leg->state_count_32 = 0;
          leg->state_count_31 = 0;
          leg->state_count_13 = 0;
          leg->TM_data = 0;   //TH 5/23/19
        }
      }

      else if ((leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref && leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)) {
        leg->state_count_21++;
        if (leg->state_count_21 >= 4 * state_counter_th) {
          if (Control_Mode == 100) {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
          }
          if (leg->Previous_Dorsi_Setpoint_Ankle <= leg->Dorsi_Setpoint_Ankle) { //Increment Dorsi Setpoint for Bang-Bang & Proportional GO - 5/19/19

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle - (leg->Previous_Dorsi_Setpoint_Ankle - leg->Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;

          }
          if (leg->New_PID_Setpoint == 0) { //GO 4/22/19
            leg->Previous_Dorsi_Setpoint_Ankle = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
          }

          leg->state_old = leg->state;
          leg->state = 1;
          leg->state_count_21 = 0;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
          leg->TM_data = 0;   //TH 5/23/19
        }
      }

      break;

    case 3: //Late Stance
      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true;
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->PID_Setpoint = 0;
        leg->Setpoint_Ankle_Pctrl = 0;
        leg->Previous_Setpoint_Ankle_Pctrl = 0; //GO 4/21/19
      }

      else if ((leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref))
      {
        leg->state_count_31++;
        if (leg->state_count_31 >= state_counter_th)
        {

          if (Control_Mode == 100) {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
          }
          if (leg->Previous_Dorsi_Setpoint_Ankle <= leg->Dorsi_Setpoint_Ankle) { //Increment Dorsi Setpoint for Bang-Bang & Proportional GO - 5/19/19

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle - (leg->Previous_Dorsi_Setpoint_Ankle - leg->Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;

          }
          if (leg->New_PID_Setpoint == 0) { //GO 4/22/19
            leg->Previous_Dorsi_Setpoint_Ankle = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
          }

          leg->state_old = leg->state;
          leg->state = 1;
          leg->state_count_31 = 0;
          leg->state_count_13 = 0;
          leg->state_count_12 = 0;
          leg->state_count_21 = 0;
          leg->allow_inc_flag = true; //TH 8/7/19
        }
      }
      else if ((leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) && (leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)) {
        if (Control_Mode != 100) {
          leg->state_count_32++;
          if (leg->state_count_32 >= state_counter_th) {
            leg->state_old = leg->state;
            leg->state = 2;
            leg->state_count_32 = 0;
            leg->state_count_23 = 0;
            leg->state_count_12 = 0;
            leg->state_count_21 = 0;
          }
        }
      }
  }//end switch
  // Adjust the torque reference as a function of the step
  ref_step_adj(leg);

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4 || Control_Mode == 6) && leg->state == 3) { //GO 4/21/19
    leg->PID_Setpoint = leg->Setpoint_Ankle_Pctrl;
  }
  
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
    }
  }

  

  if (Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4 || Control_Mode == 6) { //GO 5/19/19
    if (leg->state == 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint; //Activate Dorsiflexion during state 1
    }
    else if (leg->state == 2) {
      leg->PID_Setpoint = leg->New_PID_Setpoint; //Continue Dorsiflexion during state 2
      //leg->PID_Setpoint = 0;    //Deactivate Dorsiflexion during state 2
    }
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
    }

  }
}// end function

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void State_Machine_Heel_Toe_Sensors(Leg * leg) {

  switch (leg->state)
  {
    case 1: //Swing
      // This flag enables the "set to zero" procedure for the left ankle.
      // When you're for at least 3 seconds in the same state, the torque reference is set to zero

      if (leg->set_2_zero == 1) {
        leg->set_2_zero = 0;
        leg->One_time_set_2_zero = 1;
      }
      else if ((leg->p_steps->curr_voltage > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref))
      {
        leg->state_count_13++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_13 >= state_counter_th)
        {
          leg->sigm_done = true;
          leg->Old_PID_Setpoint = leg->PID_Setpoint;

          if (abs(leg->Dorsi_Setpoint_Ankle) > 0) {
            leg->Old_PID_Setpoint = 0;
          } else {
            leg->Previous_Dorsi_Setpoint_Ankle = 0;
          }

          if (leg->Previous_Setpoint_Ankle <= leg->Setpoint_Ankle) {

            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle - (leg->Previous_Setpoint_Ankle - leg->Setpoint_Ankle) * leg->coef_in_3_steps;

          }

          if (Flag_HLO && (leg->Previous_T_Opt <= leg->T_Opt)) {

            leg->T_Opt_Setpoint = leg->Previous_T_Opt + (leg->T_Opt - leg->Previous_T_Opt) * leg->coef_in_3_steps;

          } else if (Flag_HLO && (leg->Previous_T_Opt > leg->T_Opt)) {

            leg->T_Opt_Setpoint = leg->Previous_T_Opt - (leg->Previous_T_Opt - leg->T_Opt) * leg->coef_in_3_steps;

          }

          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
        }
      }

      break;
    case 3: //Late Stance

      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true;
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->PID_Setpoint = 0;
        leg->Setpoint_Ankle_Pctrl = 0;
      }

      if ((leg->p_steps->curr_voltage < (leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)))
      {
        leg->state_count_31++;
        if (leg->state_count_31 >= state_counter_th)
        {
          leg->sigm_done = true;
          leg->Old_PID_Setpoint = leg->PID_Setpoint;
          leg->state_old = leg->state;

          if (leg->Previous_Dorsi_Setpoint_Ankle <= leg->Dorsi_Setpoint_Ankle) {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle - (leg->Previous_Dorsi_Setpoint_Ankle - leg->Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;

          }

          leg->state = 1;
          leg->state_count_31 = 0;
          leg->state_count_13 = 0;
        }
      }
  }// end switch
  // Adjust the torque reference as a function of the step
  ref_step_adj(leg);

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4 || Control_Mode == 6) && leg->state == 3) {
    leg->PID_Setpoint = leg->Setpoint_Ankle_Pctrl;
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
    }

  }

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4 || Control_Mode == 6) && leg->state == 1) {
    leg->PID_Setpoint = 0;
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
    }

  }

}// end function


//--------------------------------------------------------------------------------------------------

void State_Machine_Heel_Toe_Sensors_Balance(Leg * leg) {
  switch (leg->state)
  {
    case 1: //Swing
      // This flag enables the "set to zero" procedure for the left ankle.
      // When you're for at least 3 seconds in the same state, the torque reference is set to zero

      if (leg->set_2_zero == 1) {
        leg->set_2_zero = 0;
        leg->One_time_set_2_zero = 1;
      }
      else if ((leg->FSR_Toe_Average > leg->fsr_percent_thresh_Toe * leg->FSR_Toe_Balance_Baseline) || (leg->FSR_Heel_Average > leg->fsr_percent_thresh_Toe * leg->FSR_Heel_Balance_Baseline))
      {
        leg->state_count_13++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_13 >= state_counter_th)
        {
          leg->sigm_done = true;
          leg->Old_PID_Setpoint = leg->PID_Setpoint;

          if (abs(leg->Dorsi_Setpoint_Ankle) > 0) {
            leg->Old_PID_Setpoint = 0;
          } else {
            leg->Previous_Dorsi_Setpoint_Ankle = 0;
          }

          if (leg->Previous_Setpoint_Ankle <= leg->Setpoint_Ankle) {

            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle - (leg->Previous_Setpoint_Ankle - leg->Setpoint_Ankle) * leg->coef_in_3_steps;

          }

          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
        }
      }

      break;
    case 3: //Late Stance

      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true;
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->PID_Setpoint = 0;
        leg->Setpoint_Ankle_Pctrl = 0;
      }

      if ((leg->FSR_Toe_Average < leg->fsr_percent_thresh_Toe * leg->FSR_Toe_Balance_Baseline) && (leg->FSR_Heel_Average < leg->fsr_percent_thresh_Toe * leg->FSR_Heel_Balance_Baseline))
      {
        leg->state_count_31++;
        if (leg->state_count_31 >= state_counter_th)
        {
          leg->sigm_done = true;
          leg->Old_PID_Setpoint = leg->PID_Setpoint;
          leg->state_old = leg->state;

          if (leg->Previous_Dorsi_Setpoint_Ankle <= leg->Dorsi_Setpoint_Ankle) {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle - (leg->Previous_Dorsi_Setpoint_Ankle - leg->Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;

          }

          leg->state = 1;
          leg->state_count_31 = 0;
          leg->state_count_13 = 0;
        }
      }
  }// end switch
  // Adjust the torque reference as a function of the step
  ref_step_adj(leg);

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4 || Control_Mode == 6) && leg->state == 3) {
    leg->PID_Setpoint = leg->Setpoint_Ankle_Pctrl;
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
    }

  }

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4 || Control_Mode == 6) && leg->state == 1) {
    leg->PID_Setpoint = 0;
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
    }

  }

}// end function




//------------------------------------------------------------



void State_Machine_Heel_Toe_Sensors_BioFeedback(Leg * leg) {

  switch (leg->state)
  {
    case 1: //Swing
      // This flag enables the "set to zero" procedure for the left ankle.
      // When you're for at least 3 seconds in the same state, the torque reference is set to zero

      if (leg->set_2_zero == 1) {
        leg->set_2_zero = 0;
        leg->One_time_set_2_zero = 1;
      }
      else if ((leg->FSR_Toe_Average > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref))
      {
        leg->state_count_13++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_13 >= state_counter_th)
        {
          leg->sigm_done = true;
          leg->Old_PID_Setpoint = leg->PID_Setpoint;

          if (abs(leg->Dorsi_Setpoint_Ankle) > 0) {
            leg->Old_PID_Setpoint = 0;
          } else {
            leg->Previous_Dorsi_Setpoint_Ankle = 0;
          }

          if (leg->Previous_Setpoint_Ankle <= leg->Setpoint_Ankle) {

            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle - (leg->Previous_Setpoint_Ankle - leg->Setpoint_Ankle) * leg->coef_in_3_steps;

          }

          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
        }
      } else if ((leg->FSR_Heel_Average > leg->fsr_percent_thresh_Toe * leg->fsr_Heel_peak_ref)) {
        // go to state 2 early stance
        leg->state_count_12++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_12 >= state_counter_th)
        {
          leg->state_old = leg->state;
          leg->state = 2;
          leg->state_count_12 = 0;

          takestridetime(leg);
        }

      }// end if heel sensor > threshold

      break;

    case 2: //Early Stance

      if (leg->state_old == 1 && leg->state == 2) {
        leg->step_count++;
      }

      if (leg->state_old = 1 && leg->step_count > 9) {
        leg->TM_data = 1;
        leg->step_count = 0;
      }

      if ((leg->FSR_Toe_Average > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref))
      {
        leg->state_count_23++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_23 >= state_counter_th)
        {
          leg->sigm_done = true;
          leg->Old_PID_Setpoint = leg->PID_Setpoint;

          if (abs(leg->Dorsi_Setpoint_Ankle) > 0) {
            leg->Old_PID_Setpoint = 0;
          } else {
            leg->Previous_Dorsi_Setpoint_Ankle = 0;
          }

          if (leg->Previous_Setpoint_Ankle <= leg->Setpoint_Ankle) {

            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle - (leg->Previous_Setpoint_Ankle - leg->Setpoint_Ankle) * leg->coef_in_3_steps;

          }

          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_count_23 = 0;
          leg->state_count_32 = 0;
          leg->TM_data = 0;
        }
      }// end if
      if ((leg->FSR_Heel_Average <= leg->fsr_percent_thresh_Toe * leg->fsr_Heel_peak_ref) && (leg->FSR_Heel_Average <= leg->fsr_percent_thresh_Toe * leg->fsr_Heel_peak_ref)) {
        // go to state 1

        leg->state_count_21++;


        if (leg->state_count_21 >= state_counter_th)
        {



          if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->state_old = leg->state;
            leg->New_PID_Setpoint = 0;
            leg->One_time_set_2_zero = 0;
            leg->Previous_Setpoint_Ankle = 0;
            leg->PID_Setpoint = 0;
            leg->Setpoint_Ankle_Pctrl = 0;
          }





          leg->sigm_done = true;
          leg->Old_PID_Setpoint = leg->PID_Setpoint;
          leg->state_old = leg->state;
          //          leg->New_PID_Setpoint = 0 * leg->coef_in_3_steps;


          if (leg->Previous_Dorsi_Setpoint_Ankle <= leg->Dorsi_Setpoint_Ankle) {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle - (leg->Previous_Dorsi_Setpoint_Ankle - leg->Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;

          }

          leg->state = 1;
          //            leg->state_count_21 = 0;
          leg->state_count_12 = 0;
          leg->TM_data = 0;

        }

      }// end if heel<th

      break;


    case 3: //Late Stance


      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true;
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->PID_Setpoint = 0;
        leg->Setpoint_Ankle_Pctrl = 0;
      }

      if ((leg->FSR_Toe_Average <= (leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->FSR_Heel_Average <= leg->fsr_percent_thresh_Toe * leg->fsr_Heel_peak_ref)))
      {
        leg->state_count_31++;
        if (leg->state_count_31 >= state_counter_th)
        {
          leg->sigm_done = true;
          leg->Old_PID_Setpoint = leg->PID_Setpoint;
          leg->state_old = leg->state;


          if (leg->Previous_Dorsi_Setpoint_Ankle <= leg->Dorsi_Setpoint_Ankle) {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle - (leg->Previous_Dorsi_Setpoint_Ankle - leg->Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;

          }

          leg->state = 1;
          leg->state_count_31 = 0;
          leg->state_count_13 = 0;
        }
      } else if ((leg->FSR_Toe_Average <= (leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->FSR_Heel_Average > leg->fsr_percent_thresh_Toe * leg->fsr_Heel_peak_ref))) {
        //          go to state 2

        leg->state_count_32++;
        if (leg->state_count_32 >= state_counter_th)
        {
          leg->sigm_done = true;
          leg->Old_PID_Setpoint = leg->PID_Setpoint;
          leg->state_old = leg->state;


          if (leg->Previous_Dorsi_Setpoint_Ankle <= leg->Dorsi_Setpoint_Ankle) {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle - (leg->Previous_Dorsi_Setpoint_Ankle - leg->Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;

          }

          leg->state = 2;
          leg->state_count_32 = 0;
          leg->state_count_23 = 0;
        }

      }//end if go to state 2

  }// end switch





  // Adjust the torque reference as a function of the step
  ref_step_adj(leg);

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4 || Control_Mode == 6) && leg->state == 3) {
    leg->PID_Setpoint = leg->Setpoint_Ankle_Pctrl;
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
    }

  }

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4 || Control_Mode == 6) && leg->state == 1) {
    leg->PID_Setpoint = 0;
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
    }

  }

}// end function

//--------------------------------------------------------------------------------------------------


void State_Machine_Heel_Toe_Sensors_Hip(Leg * leg) {

  if ((leg->state != 1)  && (leg->state != 3)  && (leg->state != 5) && (leg->state != 0))   
    leg->state = 1;
  
  switch (leg->state)
  {
    case 1: //Swing
      // This flag enables the "set to zero" procedure for the left ankle.
      // When you're for at least 3 seconds in the same state, the torque reference is set to zero
      leg->state_swing_counter++;
      leg->state_stance_counter = 0;//  SS  4/9/2021
      if (leg->set_2_zero == 1) {
        leg->set_2_zero = 0;
        leg->One_time_set_2_zero = 1;
      }
      else if ((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) || (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref)) //If the overall FSR reading is greater than the threshold we need to be in state 3
      {
        leg->state_swing_counter = 0;
        leg->state_count_13++;
        leg->SwingCon = true;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_13 >= state_counter_th)
        {
          if (abs(leg->Previous_Setpoint_Ankle_Pctrl) <= abs(leg->Setpoint_Ankle)) {
              leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle_Pctrl) * leg->coef_in_3_steps; //Increment when the new setpoint is higher than the old
            } else {
              leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl - (leg->Previous_Setpoint_Ankle_Pctrl - leg->Setpoint_Ankle) * leg->coef_in_3_steps; //Decrement when the new setpoint is lower than the old
            }
            leg->p_steps->Setpoint = leg->Setpoint_Ankle;
            if (leg->p_steps->Setpoint == 0) {
              leg->Previous_Setpoint_Ankle_Pctrl = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
            }
 

          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_stance_counter = 0;//  SS  4/9/2021
          leg->state_count_10 = 0;
          leg->state_count_03 = 0;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
          leg->state_count_35 = 0;
          leg->state_count_51 = 0;
          leg->state_count_53 = 0;
        }
      }
      else if ((leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref)  && (leg->state_swing_counter > ((leg->state_swing_duration/2)*((100-LateSwingPercentage)/100))))
      {
        leg->state_swing_counter++;
        leg->state_stance_counter = 0;//  SS  4/9/2021
        leg->state_count_10++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_10 >= state_counter_th)
        {
          leg->PID_Setpoint = -leg->Dorsi_Setpoint_Ankle; 

          leg->state_old = leg->state;
          leg->state = 0;
          leg->Min_FSR_Ratio_Hip = 0;
          leg->state_count_10 = 0;
          leg->state_count_03 = 0;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
          leg->state_count_35 = 0;
          leg->state_count_51 = 0;
          leg->state_count_53 = 0;
        }
      }

      break;

    case 3: //Late Stance
      leg->state_swing_counter = 0;
      leg->state_stance_counter++;//  SS  4/9/2021
      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true;
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->PID_Setpoint = 0;
        leg->Setpoint_Ankle_Pctrl = 0;
        leg->Previous_Setpoint_Ankle_Pctrl = 0; //GO 4/21/19
      }

      else if ((leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref)  && (leg->state_swing_counter > ((leg->state_swing_duration/2)*(EarlySwingPercentage/100))) && (leg->state_swing_counter <= ((leg->state_swing_duration/2)*((100-LateSwingPercentage)/100))) )
      {
        leg->state_swing_counter++;
        leg->state_stance_counter = 0;//  SS  4/9/2021
        leg->state_count_31++;
        if (leg->state_count_31 >= state_counter_th)
        {
          if (leg->Previous_Dorsi_Setpoint_Ankle <= leg->Dorsi_Setpoint_Ankle) { //Increment Dorsi Setpoint for Bang-Bang & Proportional GO - 5/19/19
            leg->New_PID_Setpoint = 0;
          } else {
            leg->New_PID_Setpoint = 0;
          }
          if (leg->New_PID_Setpoint == 0) { //GO 4/22/19
            leg->Previous_Dorsi_Setpoint_Ankle = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
          }

          leg->state_old = leg->state;
          leg->state = 1;
          leg->Min_FSR_Ratio_Hip = 0;
          leg->state_count_10 = 0;
          leg->state_count_03 = 0;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
          leg->state_count_35 = 0;
          leg->state_count_51 = 0;
          leg->state_count_53 = 0;
          leg->allow_inc_flag = true; //TH 8/7/19
        }
      }
      else if( (leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)  &&  (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) && (leg->state_swing_counter <= ((leg->state_swing_duration/2)*(EarlySwingPercentage/100))) ) //LKL 9/8/2020
      {
        leg->state_count_35++;
        leg->state_swing_counter++;
        leg->state_stance_counter = 0;//  SS  4/9/2021
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_35 >= state_counter_th)
        {
          leg->sigm_done = true;
//          leg->Old_PID_Setpoint = leg->PID_Setpoint;
//          leg->New_PID_Setpoint = leg->Dorsi_Setpoint_Ankle;//(leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps);
            if (HeelMToe)
              leg->PID_Setpoint = leg->Min_FSR_Ratio_Hip * leg->p_steps->Setpoint;
            else if (HeelMToe4)
              leg->PID_Setpoint = 4 * leg->Min_FSR_Ratio_Hip * leg->p_steps->Setpoint;
            else
              leg->PID_Setpoint = -leg->p_steps->Setpoint;
            
          leg->state_old = leg->state;
          leg->state = 5;
          leg->state_count_10 = 0;
          leg->state_count_03 = 0;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
          leg->state_count_35 = 0;
          leg->state_count_51 = 0;
          leg->state_count_53 = 0;
        }
      }
      break;

    case 5: //Early Swing
      leg->state_swing_counter++;
      leg->state_stance_counter = 0;//  SS  4/9/2021
      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true;
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->PID_Setpoint = 0;
        leg->Setpoint_Ankle_Pctrl = 0;
        leg->Previous_Setpoint_Ankle_Pctrl = 0; 
      }

      else if ((leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref)  && (leg->state_swing_counter > ((leg->state_swing_duration/2)*(EarlySwingPercentage/100))) && (leg->state_swing_counter <= ((leg->state_swing_duration/2)*(100-LateSwingPercentage)/100))) //LKL 9/8/2020  
      {
        leg->state_count_51++;
        leg->state_swing_counter++;
        leg->state_stance_counter = 0;//  SS  4/9/2021
        if (leg->state_count_51 >= state_counter_th)
        {
          leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = 0;
            leg->PID_Setpoint = 0;
          
          leg->state_old = leg->state;
          leg->state = 1;
          leg->Min_FSR_Ratio_Hip = 0;
          leg->state_count_10 = 0;
          leg->state_count_03 = 0;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
          leg->state_count_35 = 0;
          leg->state_count_51 = 0;
          leg->state_count_53 = 0;
        }
      }
      else if(( (leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)  ||  (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) )  && (leg->state_swing_counter < ((leg->state_swing_duration/2)*(EarlySwingPercentage/100)))) //LKL 9/8/2020
      {
        leg->state_swing_counter = 0;
        leg->state_stance_counter = 0;//  SS  4/9/2021
        leg->state_old = leg->state;
        leg->state = 1;
      }else if ((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) || (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref)) //If the overall FSR reading is greater than the threshold we need to be in state 3
      {
        leg->state_swing_counter = 0;
        leg->state_count_53++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_53 >= state_counter_th)
        {
          if (abs(leg->Previous_Setpoint_Ankle_Pctrl) <= abs(leg->Setpoint_Ankle)) {
              leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle_Pctrl) * leg->coef_in_3_steps; //Increment when the new setpoint is higher than the old
            } else {
              leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl - (leg->Previous_Setpoint_Ankle_Pctrl - leg->Setpoint_Ankle) * leg->coef_in_3_steps; //Decrement when the new setpoint is lower than the old
            }
            leg->p_steps->Setpoint = leg->Setpoint_Ankle;
            if (leg->p_steps->Setpoint == 0) {
              leg->Previous_Setpoint_Ankle_Pctrl = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
            }
 

          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_stance_counter = 0;//  SS  4/9/2021
          leg->state_count_10 = 0;
          leg->state_count_03 = 0;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
          leg->state_count_35 = 0;
          leg->state_count_51 = 0;
          leg->state_count_53 = 0;
        }
      }
      break;
      
    case 0: //Swing
      // This flag enables the "set to zero" procedure for the left ankle.
      // When you're for at least 3 seconds in the same state, the torque reference is set to zero
      leg->state_swing_counter++;
      leg->state_stance_counter = 0;//  SS  4/9/2021
      if (leg->set_2_zero == 1) {
        leg->set_2_zero = 0;
        leg->One_time_set_2_zero = 1;
      }
      else if ((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) || (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref)) //If the overall FSR reading is greater than the threshold we need to be in state 3
      {
        leg->state_swing_counter = 0;//  SS  4/9/2021
        leg->state_count_03++;
        leg->SwingCon = true;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_03 >= state_counter_th)
        {
          if (abs(leg->Previous_Setpoint_Ankle_Pctrl) <= abs(leg->Setpoint_Ankle)) {
              leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle_Pctrl) * leg->coef_in_3_steps; //Increment when the new setpoint is higher than the old
            } else {
              leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl - (leg->Previous_Setpoint_Ankle_Pctrl - leg->Setpoint_Ankle) * leg->coef_in_3_steps; //Decrement when the new setpoint is lower than the old
            }
            leg->p_steps->Setpoint = leg->Setpoint_Ankle;
            if (leg->p_steps->Setpoint == 0) {
              leg->Previous_Setpoint_Ankle_Pctrl = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
            }
 

          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_stance_counter = 0;//  SS  4/9/2021
          leg->state_count_10 = 0;
          leg->state_count_03 = 0;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
          leg->state_count_35 = 0;
          leg->state_count_51 = 0;
          leg->state_count_53 = 0;
        }
      }
      
  }//end switch
  // Adjust the torque reference as a function of the step
  ref_step_adj(leg);

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4 || Control_Mode == 6) && (leg->state == 3) && (leg->state_stance_counter > 0)) { //GO 4/21/19  //  SS  4/9/2021
    leg->PID_Setpoint = leg->Setpoint_Ankle_Pctrl;
  }
  
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
    }
//    else if (leg->state == 5){
//      // Create the smoothed reference and call the PID
//      PID_Sigm_Curve(leg);
//    }
  }

}// end function
//--------------------------------------------------------------------------------------------------


void State_Machine_Heel_Toe_Sensors_Hip_3State(Leg * leg) {

  if ((leg->state != 1)  && (leg->state != 3)  && (leg->state != 5)){   
    leg->state = 1;
    leg->PID_Setpoint = 0;
  }
  
  switch (leg->state)
  {
    case 1: //Swing
      // This flag enables the "set to zero" procedure for the left ankle.
      // When you're for at least 3 seconds in the same state, the torque reference is set to zero
      leg->state_swing_counter++;
      leg->state_1_counter++;
      leg->state_stance_counter = 0;//  SS  4/9/2021
      if (HeelMToe){
        leg->PID_Setpoint = leg->PID_Setpoint + (((-leg->Dorsi_Setpoint_Ankle * 0.6) - (leg->Old_PID_Setpoint)) / leg->prev_state_swing_counter);
        if (leg->Dorsi_Setpoint_Ankle * 0.6 < 0){
        if ((leg->PID_Setpoint) > (-leg->Dorsi_Setpoint_Ankle * 0.6))
          leg->PID_Setpoint = -leg->Dorsi_Setpoint_Ankle * 0.6;
      }else{
        if ((leg->PID_Setpoint) < (-leg->Dorsi_Setpoint_Ankle * 0.6))
          leg->PID_Setpoint = -leg->Dorsi_Setpoint_Ankle * 0.6;
      }
      }else if (HeelMToe4){
        leg->PID_Setpoint = leg->PID_Setpoint + (((-leg->Dorsi_Setpoint_Ankle * 0.6) - (leg->Old_PID_Setpoint)) / leg->prev_state_swing_counter);
        if (leg->Dorsi_Setpoint_Ankle * 0.6 < 0){
        if ((leg->PID_Setpoint) > (-leg->Dorsi_Setpoint_Ankle * 0.6))
          leg->PID_Setpoint = -leg->Dorsi_Setpoint_Ankle * 0.6;
      }else{
        if ((leg->PID_Setpoint) < (-leg->Dorsi_Setpoint_Ankle * 0.6))
          leg->PID_Setpoint =-leg->Dorsi_Setpoint_Ankle * 0.6;
      }
      }else{
       leg->PID_Setpoint = leg->PID_Setpoint + (((-leg->Dorsi_Setpoint_Ankle * 0.6) - (leg->Old_PID_Setpoint)) / leg->prev_state_swing_counter);
       if (leg->Dorsi_Setpoint_Ankle * 0.6 < 0){
        if ((leg->PID_Setpoint) > (-leg->Dorsi_Setpoint_Ankle * 0.6))
          leg->PID_Setpoint = -leg->Dorsi_Setpoint_Ankle * 0.6;
      }else{
        if ((leg->PID_Setpoint) < (-leg->Dorsi_Setpoint_Ankle * 0.6))
          leg->PID_Setpoint = -leg->Dorsi_Setpoint_Ankle * 0.6;
      }
      }
      if (abs(leg->PID_Setpoint) > abs(10 * leg->Setpoint_Ankle))
          leg->PID_Setpoint = 0;
      
      if (leg->set_2_zero == 1) {
        leg->set_2_zero = 0;
        leg->One_time_set_2_zero = 1;
      }
      else if ((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) || (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref)) //If the overall FSR reading is greater than the threshold we need to be in state 3
      {
        leg->state_swing_counter = 0;
        if(leg->state_1_counter > state_counter_th)
          leg->prev_state_swing_counter = leg->state_1_counter;
        leg->state_1_counter = 0;
        leg->state_count_13++;
        leg->SwingCon = true;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_13 >= state_counter_th)
        {
          if (abs(leg->Previous_Setpoint_Ankle_Pctrl) <= abs(leg->Setpoint_Ankle)) {
              leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle_Pctrl) * leg->coef_in_3_steps; //Increment when the new setpoint is higher than the old
            } else {
              leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl - (leg->Previous_Setpoint_Ankle_Pctrl - leg->Setpoint_Ankle) * leg->coef_in_3_steps; //Decrement when the new setpoint is lower than the old
            }
            leg->p_steps->Setpoint = leg->Setpoint_Ankle;
            if (leg->p_steps->Setpoint == 0) {
              leg->Previous_Setpoint_Ankle_Pctrl = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
            }
 

          leg->state_old = leg->state;
          leg->state = 3;
          leg->Min_FSR_Ratio_Hip = 0;
          leg->state_stance_counter = 0;//  SS  4/9/2021
          leg->state_count_10 = 0;
          leg->state_count_03 = 0;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
          leg->state_count_35 = 0;
          leg->state_count_51 = 0;
          leg->state_count_53 = 0;
        }
      }
      break;

    case 3: //Late Stance
      leg->state_swing_counter = 0;
      leg->state_1_counter = 0;
      leg->state_stance_counter++;//  SS  4/9/2021
      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true;
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->PID_Setpoint = 0;
        leg->Setpoint_Ankle_Pctrl = 0;
        leg->Previous_Setpoint_Ankle_Pctrl = 0; //GO 4/21/19
      }

      else if ((leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref)  && (leg->state_swing_counter > ((leg->state_swing_duration/2)*(EarlySwingPercentage/100))) && (leg->state_swing_counter <= ((leg->state_swing_duration/2)*((100-LateSwingPercentage)/100))) )
      {
        leg->state_swing_counter++;
        leg->state_1_counter++;
        leg->state_stance_counter = 0;//  SS  4/9/2021
        leg->state_count_31++;
        if (leg->state_count_31 >= state_counter_th)
        {
          if (leg->Previous_Dorsi_Setpoint_Ankle <= leg->Dorsi_Setpoint_Ankle) { //Increment Dorsi Setpoint for Bang-Bang & Proportional GO - 5/19/19
            leg->New_PID_Setpoint = 0;
          } else {
            leg->New_PID_Setpoint = 0;
          }
          if (leg->New_PID_Setpoint == 0) { //GO 4/22/19
            leg->Previous_Dorsi_Setpoint_Ankle = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
          }

          leg->state_old = leg->state;
          leg->state = 1;
          leg->state_count_10 = 0;
          leg->state_count_03 = 0;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
          leg->state_count_35 = 0;
          leg->state_count_51 = 0;
          leg->state_count_53 = 0;
          leg->allow_inc_flag = true; //TH 8/7/19
        }
      }
      else if( (leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)  &&  (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) && (leg->state_swing_counter <= ((leg->state_swing_duration/2)*(EarlySwingPercentage/100))) ) //LKL 9/8/2020
      {
        leg->state_count_35++;
        leg->state_1_counter = 0;
        leg->state_swing_counter++;
        leg->state_stance_counter = 0;//  SS  4/9/2021
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_35 >= state_counter_th)
        {
          leg->sigm_done = true;
          if (leg->lateStance_counter != 0){  //  SS  6/7/2021
            leg->Pre_lateStance_duration = leg->lateStance_duration;
            leg->lateStance_duration = leg->lateStance_counter;
            leg->lateStance_counter = 0;
          } 
            
          leg->state_old = leg->state;
          leg->state = 5;
          leg->state_count_10 = 0;
          leg->state_count_03 = 0;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
          leg->state_count_35 = 0;
          leg->state_count_51 = 0;
          leg->state_count_53 = 0;
        }
      }
      break;

    case 5: //Early Swing
      leg->state_swing_counter++;
      leg->state_1_counter = 0;
      leg->state_stance_counter = 0;//  SS  4/9/2021
      leg->Alpha_counter = leg->state_swing_counter + leg->lateStance_duration;
//      leg->Alpha = ((leg->state_swing_duration/4)*(EarlySwingPercentage/100)) + (leg->Pre_lateStance_duration / 2);
      leg->Alpha = ((leg->state_swing_duration/2)*(EarlySwingPercentage/100)) + (leg->Pre_lateStance_duration);

      if ((((9 * pow(leg->Alpha_counter / leg->Alpha,2))-(9 * leg->Alpha_counter / leg->Alpha)) / ((3 * leg->Alpha_counter / leg->Alpha) - 4)) < 0){//Switch to state 1
        leg->state_swing_counter++;
        leg->state_1_counter++;
        leg->state_stance_counter = 0;//  SS  4/9/2021
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->New_PID_Setpoint = 0;
          
        leg->state_old = leg->state;
        leg->state = 1;
        leg->state_count_10 = 0;
        leg->state_count_03 = 0;
        leg->state_count_13 = 0;
        leg->state_count_31 = 0;
        leg->state_count_35 = 0;
        leg->state_count_51 = 0;
        leg->state_count_53 = 0;
      }

      
      if (HeelMToe){
        leg->PID_Setpoint = ((leg->Min_FSR_Ratio_Hip * 0.50) - (0.50 * (((9 * pow(leg->Alpha_counter / leg->Alpha,2))-(9 * leg->Alpha_counter / leg->Alpha)) / ((3 * leg->Alpha_counter / leg->Alpha) - 4)))) * leg->Setpoint_Ankle;
      }else if(HeelMToe4){
        leg->PID_Setpoint = ((4 * leg->Min_FSR_Ratio_Hip * 0.50) - (0.50 * (((9 * pow(leg->Alpha_counter / leg->Alpha,2))-(9 * leg->Alpha_counter / leg->Alpha)) / ((3 * leg->Alpha_counter / leg->Alpha) - 4)))) * leg->Setpoint_Ankle;
      }else{
        leg->PID_Setpoint = ((-0.50) - (0.50 * (((9 * pow(leg->Alpha_counter / leg->Alpha,2))-(9 * leg->Alpha_counter / leg->Alpha)) / ((3 * leg->Alpha_counter / leg->Alpha) - 4)))) * leg->Setpoint_Ankle;
      }
      if (abs(leg->PID_Setpoint) > abs(10 * leg->Setpoint_Ankle))
          leg->PID_Setpoint = 0;
      
      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true;
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->PID_Setpoint = 0;
        leg->Setpoint_Ankle_Pctrl = 0;
        leg->Previous_Setpoint_Ankle_Pctrl = 0; 
      }

      else if ((leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref)  && (leg->state_swing_counter > ((leg->state_swing_duration/2)*(EarlySwingPercentage/100))) && (leg->state_swing_counter <= ((leg->state_swing_duration/2)*(100-LateSwingPercentage)/100))) //LKL 9/8/2020  
      {
        leg->state_count_51++;
        leg->state_swing_counter++;
        leg->state_1_counter++;
        leg->state_stance_counter = 0;//  SS  4/9/2021
        if (leg->state_count_51 >= state_counter_th)
        {
          
          leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = 0;
          
          leg->state_old = leg->state;
          leg->state = 1;
          leg->state_count_10 = 0;
          leg->state_count_03 = 0;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
          leg->state_count_35 = 0;
          leg->state_count_51 = 0;
          leg->state_count_53 = 0;
        }
      }
      else if(( (leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)  ||  (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) )  && (leg->state_swing_counter < ((leg->state_swing_duration/2)*(EarlySwingPercentage/100)))) //LKL 9/8/2020
      {
        leg->state_swing_counter = 0;
        leg->state_stance_counter = 0;//  SS  4/9/2021
        leg->state_1_counter = 0;
        leg->state_old = leg->state;
        leg->state = 1;
      }else if ((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) || (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref)) //If the overall FSR reading is greater than the threshold we need to be in state 3
      {
        leg->state_swing_counter = 0;
        leg->state_1_counter = 0;
        leg->state_count_53++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_13 >= state_counter_th)
        {
          if (abs(leg->Previous_Setpoint_Ankle_Pctrl) <= abs(leg->Setpoint_Ankle)) {
              leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle_Pctrl) * leg->coef_in_3_steps; //Increment when the new setpoint is higher than the old
            } else {
              leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl - (leg->Previous_Setpoint_Ankle_Pctrl - leg->Setpoint_Ankle) * leg->coef_in_3_steps; //Decrement when the new setpoint is lower than the old
            }
            leg->p_steps->Setpoint = leg->Setpoint_Ankle;
            if (leg->p_steps->Setpoint == 0) {
              leg->Previous_Setpoint_Ankle_Pctrl = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
            }
 

          leg->state_old = leg->state;
          leg->state = 3;
          leg->Min_FSR_Ratio_Hip = 0;
          leg->state_stance_counter = 0;//  SS  4/9/2021
          leg->state_count_10 = 0;
          leg->state_count_03 = 0;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
          leg->state_count_35 = 0;
          leg->state_count_51 = 0;
          leg->state_count_53 = 0;
        }
      }
      
  }//end switch
  // Adjust the torque reference as a function of the step
  ref_step_adj(leg);
  
  if ((abs(leg->PID_Setpoint) > abs(5 * leg->Setpoint_Ankle)) && (abs(leg->PID_Setpoint) > abs(5 * leg->Dorsi_Setpoint_Ankle)))
          leg->PID_Setpoint = 0;

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4 || Control_Mode == 6) && (leg->state == 3) && (leg->state_stance_counter > 0)) { //GO 4/21/19  //  SS  4/9/2021
    leg->PID_Setpoint = leg->Setpoint_Ankle_Pctrl;
  }
  
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
    }
//    else if (leg->state == 5){
//      // Create the smoothed reference and call the PID
//      PID_Sigm_Curve(leg);
//    }
  }
//  if ((leg->state == 5) && (Heel || HeelPToe) ){
      // Create the smoothed reference and call the PID
//      PID_Sigm_Curve(leg);
//    }

}// end function
//-------------------------------------------------------------------------------------------------------------
void State_Machine_BangBang_Hip(Leg * leg) {  // SS 11/17/2020
  switch (leg->state)
  {
    if (leg->state == 0) //  SS  4/9/2021
      leg->state = 1;
      
    case 1: //Late Swing
      // This flag enables the "set to zero" procedure for the left ankle.
      // When you're for at least 3 seconds in the same state, the torque reference is set to zero
      leg->state_swing_counter++; //  SS  11/4/2020
      if (leg->set_2_zero == 1) {
        leg->set_2_zero = 0;
        leg->One_time_set_2_zero = 1;
      }

      //transition from state 1 to state 2
      else if( (leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)  &&  (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) )// Transition from state 1 to state 2// SS 1/27/2020
      {
        leg->state_count_12++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_12 >= state_counter_th)
        {
          
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = 0;
            leg->PID_Setpoint = 0;

          
          leg->state_old = leg->state;
          leg->state = 2;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0;
          leg->state_count_21 = 0; 
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0;
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0;
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0; 
          leg->state_swing_counter = 0;
        }
      }

      //transition from state 1 to state 3
      else if (((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref))) //LKL 9/8/2020
      {
        leg->state_count_13++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_13 >= state_counter_th)
        {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            if (leg->Previous_Setpoint_Ankle <= leg->Setpoint_Ankle)
              leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps;
            else
              leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle - (leg->Previous_Setpoint_Ankle - leg->Setpoint_Ankle) * leg->coef_in_3_steps;

            if (Flag_HLO && (leg->Previous_T_Opt <= leg->T_Opt))
              leg->T_Opt_Setpoint = leg->Previous_T_Opt + (leg->T_Opt - leg->Previous_T_Opt) * leg->coef_in_3_steps;
            else if (Flag_HLO && (leg->Previous_T_Opt > leg->T_Opt))
              leg->T_Opt_Setpoint = leg->Previous_T_Opt - (leg->Previous_T_Opt - leg->T_Opt) * leg->coef_in_3_steps;
          
          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0;
          leg->state_count_21 = 0; 
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0; 
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0; 
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0; 
          leg->state_swing_counter = 0;
        }
      }


      //transition from state 1 to state 4
      else if (((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref))) //LKL 9/8/2020
      {
        leg->state_count_14++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_14 >= state_counter_th)
        {
          
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = (leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps)/2;

          leg->state_old = leg->state;
          leg->state = 4;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0;
          leg->state_count_21 = 0;
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0; 
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0; 
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0; 
          leg->state_swing_counter = 0;
        }
      }
      
      break;

    case 2: //Early Stance // SS 1/27/2020
      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true; 
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->PID_Setpoint = 0;
        leg->Setpoint_Ankle_Pctrl = 0;
        leg->Previous_Setpoint_Ankle_Pctrl = 0;
        }
  
      //transition from state 2 to state 1
      else if ((leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) && (leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->state_swing_counter > ((leg->state_swing_duration/2)*(EarlySwingPercentage/100))) ) {// Transition from state 2 to state 1// SS 1/27/2020   
        leg->state_swing_counter++;
        leg->state_count_21++;
        if (leg->state_count_21 >= 4 * state_counter_th) 
        {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = 0;
            leg->PID_Setpoint = 0;
         
          leg->state_old = leg->state;
          leg->state = 1;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0;
          leg->state_count_21 = 0; 
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0; 
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0; 
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0; 
        }
      }

      //transition from state 2 to state 3
      else if (((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref))) { //LKL 9/8/2020
        leg->state_count_23++;
        if (leg->state_count_23 >= state_counter_th) 
        {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = (leg->Previous_Setpoint_Ankle + ((leg->Setpoint_Ankle) - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps);
          
          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0;
          leg->state_count_21 = 0; 
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0;
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0;
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0; 
          leg->state_swing_counter = 0;
        }
      }

      //transition from state 2 to state 4
      else if (((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref))) { //LKL 9/8/2020
        leg->state_count_24++;
        if (leg->state_count_24 >= state_counter_th) 
        {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = (leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps)/2;
          
          leg->state_old = leg->state;
          leg->state = 4;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0; 
          leg->state_count_21 = 0; 
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0;
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0;
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0; 
          leg->state_swing_counter = 0;
        }
      }

      //transition from state 2 to state 5
      //IDK HOW TO DO THIS ONE, USE SOME TIMER TO DETERMINE WHEN TO SWITCH TO STATE 1
      else if ((leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) && (leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)  && (leg->state_swing_counter <= ((leg->state_swing_duration/2)*(EarlySwingPercentage/100)))) { //LKL 9/8/2020  
        leg->state_count_25++;
        leg->state_swing_counter++;
        if (leg->state_count_25 >= state_counter_th) 
        {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = (leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps);
          
          
          leg->state_old = leg->state;
          leg->state = 5;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0;
          leg->state_count_21 = 0; 
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0;
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0; 
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0; 
        }
      }
      break;

    case 3: //Mid Stance
      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true;
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->PID_Setpoint = 0;
        leg->Setpoint_Ankle_Pctrl = 0;
        leg->Previous_Setpoint_Ankle_Pctrl = 0; 
      }

      //Transition from state 3 to state 1
      else if ((leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref)  && (leg->state_swing_counter > ((leg->state_swing_duration/2)*(EarlySwingPercentage/100)))) //LKL 9/8/2020   
      {
        leg->state_swing_counter++;
        leg->state_count_31++;
        if (leg->state_count_31 >= state_counter_th)
        {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = 0;
            leg->PID_Setpoint = 0;
         
          leg->state_old = leg->state;
          leg->state = 1;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0;
          leg->state_count_21 = 0; 
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0;
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0;
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0; 
        }
      }

      //Transition from state 3 to state 2
      else if( (leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)  &&  (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) ) //LKL 9/8/2020
      {
        leg->state_count_32++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_32 >= state_counter_th)
        {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = 0;
            leg->PID_Setpoint = 0;
            
          leg->state_old = leg->state;
          leg->state = 2;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0;
          leg->state_count_21 = 0; 
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0; 
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0; 
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0; 
          leg->state_swing_counter = 0; //  SS  11/4/2020
        }
      }

      //Transition from state 3 to state 4
      else if( (leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)  &&  (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) ) //LKL 9/8/2020
      {
        leg->state_count_34++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_34 >= state_counter_th)
        {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = (leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps)/2;
//            leg->Old_PID_Setpoint = 0.01 * leg->New_PID_Setpoint;
          leg->state_old = leg->state;
          leg->state = 4;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0;
          leg->state_count_21 = 0;
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0; 
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0; 
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0; 
          leg->state_swing_counter = 0; //  SS  11/4/2020
        }
      }

      //Transition from state 3 to state 5
      //IDK HOW TO DO THIS ONE
      else if( (leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)  &&  (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) && (leg->state_swing_counter <= ((leg->state_swing_duration/2)*(EarlySwingPercentage/100))) ) //LKL 9/8/2020
      {
        leg->state_count_35++;
        leg->state_swing_counter++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_35 >= state_counter_th)
        {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = (leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps);
          
          leg->state_old = leg->state;
          leg->state = 5;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0;
          leg->state_count_21 = 0; 
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0; 
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0; 
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0; 
        }
      }
    break;

    case 4: //Late Stance
      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true;
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->PID_Setpoint = 0;
        leg->Setpoint_Ankle_Pctrl = 0;
        leg->Previous_Setpoint_Ankle_Pctrl = 0; 
      }

      //Transition from state 4 to state 1
      else if ((leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) && (leg->state_swing_counter > ((leg->state_swing_duration/2)*(EarlySwingPercentage/100))))     // 
      {
        leg->state_count_41++;
        leg->state_swing_counter++;
        if (leg->state_count_41 >= state_counter_th)
        {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = 0;
            leg->PID_Setpoint = 0;
          
          leg->state_old = leg->state;
          leg->state = 1;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0;
          leg->state_count_21 = 0; 
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0; 
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0; 
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0; 
        }
      }

      //Transition from state 4 to state 2
      else if( (leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)  &&  (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) ) //LKL 9/8/2020
      {
        leg->state_count_42++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_42 >= state_counter_th)
        {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = 0;
            leg->PID_Setpoint = 0;
          
          leg->state_old = leg->state;
          leg->state = 2;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0;
          leg->state_count_21 = 0;
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0;
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0; 
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0; 
          leg->state_swing_counter = 0; //  SS  11/4/2020
        }
      }

      //Transition from state 4 to state 3
      else if( (leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)  &&  (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) ) //LKL 9/8/2020
      {
        leg->state_count_43++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_43 >= state_counter_th)
        {
           leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            if (leg->Previous_Setpoint_Ankle <= leg->Setpoint_Ankle)
              leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps;
            else
              leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle - (leg->Previous_Setpoint_Ankle - leg->Setpoint_Ankle) * leg->coef_in_3_steps;

            if (Flag_HLO && (leg->Previous_T_Opt <= leg->T_Opt))
              leg->T_Opt_Setpoint = leg->Previous_T_Opt + (leg->T_Opt - leg->Previous_T_Opt) * leg->coef_in_3_steps;
            else if (Flag_HLO && (leg->Previous_T_Opt > leg->T_Opt))
              leg->T_Opt_Setpoint = leg->Previous_T_Opt - (leg->Previous_T_Opt - leg->T_Opt) * leg->coef_in_3_steps;
          
          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0;
          leg->state_count_21 = 0; 
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0; 
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0;
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0; 
          leg->state_swing_counter = 0; //  SS  11/4/2020
        }
      }

      //Transition from state 4 to state 5
      //IDK HOW TO DO THIS ONE
      else if( (leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)  &&  (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) && (leg->state_swing_counter <= ((leg->state_swing_duration/2)*(EarlySwingPercentage/100))) ) //LKL 9/8/2020
      {
        leg->state_count_45++;
        leg->state_swing_counter++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_45 >= state_counter_th)
        {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = (leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps);
          
          leg->state_old = leg->state;
          leg->state = 5;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0;
          leg->state_count_21 = 0;
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0;
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0; 
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0; 
        }
      }
    break;

    case 5: //Early Swing
      leg->state_swing_counter++;
      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true;
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->PID_Setpoint = 0;
        leg->Setpoint_Ankle_Pctrl = 0;
        leg->Previous_Setpoint_Ankle_Pctrl = 0; 
      }

      else if ((leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref)  && (leg->state_swing_counter > ((leg->state_swing_duration/2)*(EarlySwingPercentage/100)))) //LKL 9/8/2020  
      {
        leg->state_count_51++;
        leg->state_swing_counter++;
        if (leg->state_count_51 >= state_counter_th)
        {
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->New_PID_Setpoint = 0;
            leg->PID_Setpoint = 0;
          
          leg->state_old = leg->state;
          leg->state = 1;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_14 = 0;
          leg->state_count_21 = 0;
          leg->state_count_23 = 0;
          leg->state_count_24 = 0;
          leg->state_count_25 = 0;
          leg->state_count_31 = 0; 
          leg->state_count_32 = 0;
          leg->state_count_34 = 0;
          leg->state_count_35 = 0;
          leg->state_count_41 = 0; 
          leg->state_count_42 = 0;
          leg->state_count_43 = 0;
          leg->state_count_45 = 0;
          leg->state_count_51 = 0;
        }
      }else if (((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) || (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref))  && (leg->state_swing_counter < ((leg->state_swing_duration/2)*(EarlySwingPercentage/100)))) //LKL 9/8/2020  
      {
        leg->state_swing_counter++;
        leg->state_old = leg->state;
        leg->state = 1;
      }


  }//end switch
  // Adjust the torque reference as a function of the step

  ref_step_adj(leg);

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
    }else if ((leg->state == 3) || (leg->state == 4) || (leg->state == 5)){
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
    }

}// end function
