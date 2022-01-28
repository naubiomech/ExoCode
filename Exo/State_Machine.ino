
//--------------------NEW FUNCTIONS-----------------
// Before state 3 in case of balance control was activated just in case of not negligible value
// of the force on the toe i.e. if you just walk on the heel you never feel the balance.
// Right now the state 3 is activate also in case of heel contact and the state 1 is activated
// in case of no contact of both the sensors
void state_machine(Leg* leg)
{
  if (FLAG_ONE_TOE_SENSOR) {
    State_Machine_One_Toe_Sensor(leg);
  }

  else if (FLAG_BALANCE) {
    //State_Machine_Heel_Toe_Sensors_Balance(leg);
  } else {
    if (FLAG_BIOFEEDBACK) {
      //State_Machine_Heel_Toe_Sensors_BioFeedback(leg);
    } else {
      //State_Machine_Heel_Toe_Sensors(leg);
    }
  }
}
// P tunings to explore 
void P_values(int old_best_P, int old_range, int P_vec[3]){
  int range = round(0.5 * old_range);
  //P_vec[] = {old_best_P - range, old_best_P, old_best_P + range};
  P_vec[0] = old_best_P - range;
  P_vec[1] = old_best_P;
  P_vec[2] = old_best_P + range;
}

// update state values
void update_state_value(int step_count_start, int step_count_current, int P_vec[3], int P_error_vec[3], int P_error_count[3]){
  
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
      else if (leg->p_steps->curr_voltage_Toe > (leg->fsr_Toe_trough_ref + (leg->fsr_percent_thresh_Toe * (leg->fsr_Toe_peak_ref - leg->fsr_Toe_trough_ref)))) //If the overall FSR reading is greater than the threshold we need to be in state 3
      {
        leg->state_count_13++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_13 >= state_counter_th)
        {
          if (stream) {
            //Reference time to be used later for step counter
            if (leg == right_leg) {
              stepper->right_step_start = millis();
            }
            else {
              stepper->left_step_start = millis();
            }
          }

          if (Control_Mode == 100) { //Increment set point for bang-bang GO - 5/19/19
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->d_counter++;                                 //kh 11/2021
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
          leg->Max_FSR_Ratio = 0;
          leg->pid.SetTunings(300, 0, 3); 
          leg->d_counter =0;
        }
        
      }
      leg->pid.SetTunings(375, 0, 3); //+ leg->d_counter);   //kh 12/21 incriment the derivative to mitigate increasing error at heal strike?
      //leg->d_counter = leg->d_counter +1;
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

      else if (leg->p_steps->curr_voltage_Toe <= (leg->fsr_Toe_trough_ref + (leg->fsr_percent_thresh_Toe * (leg->fsr_Toe_peak_ref - leg->fsr_Toe_trough_ref))))
      {
        leg->state_count_31++;
        if (leg->state_count_31 >= state_counter_th)
        {
          //Check if stance phase was long enough to warrant a valid step
          if (stream) {
            if (leg == right_leg) {
              uint32_t delta = (millis() - stepper->right_step_start);
              bool condition = (delta > stepper->step_delay_k) && (delta < stepper->step_limit_k);
              stepper->steps += (condition) ? (1):(0);
            }
            else {
              uint32_t delta = (millis() - stepper->left_step_start);
              bool condition = (delta > stepper->step_delay_k) && (delta < stepper->step_limit_k);
              stepper->steps += (condition) ? (1):(0);
            }
          }
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
          leg->swing_counter = 0;
          leg->allow_inc_flag = true; //TH 8/7/19

          leg->pid.SetTunings(375, 0, 3);   //kh 11/21
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
      if (leg->swing_counter <= swing_counter_th*leg->Max_FSR_Ratio) { //Max FSR Ratio scales DF peak duration for speed adaptation
        leg->PID_Setpoint = 2*leg->New_PID_Setpoint*leg->Max_FSR_Ratio; //Activate Dorsiflexion during state 1. Max FSR Ratio scales DF peak for speed adaptation
      } else {
        leg->PID_Setpoint = leg->New_PID_Setpoint; //Activate Dorsiflexion during state 1
      }
      leg->swing_counter++;
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
