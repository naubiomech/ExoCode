#include <deque>
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

          leg->stance_times.push_back(millis());
          if(leg->stance_times.size() >= (leg->time_array_length)) {
            leg->stance_times.pop_front();
            leg->full_times_array = true;
          }

          if(!FSR_CAL_FLAG && leg->full_times_array && ((leg->stance_times[leg->time_array_length-1] - leg->stance_times[0]) < 750)) {
            
            right_leg->Previous_Setpoint_Ankle = 0;
            right_leg->Previous_Setpoint_Ankle_Pctrl = 0;
            right_leg->Previous_Dorsi_Setpoint_Ankle = 0;
            right_leg->first_step = 1;
            right_leg->activate_in_3_steps = 1;
            right_leg->coef_in_3_steps = 0;
            right_leg->start_step = 0;
            
            left_leg->Previous_Setpoint_Ankle = 0;
            left_leg->Previous_Setpoint_Ankle_Pctrl = 0;
            left_leg->Previous_Dorsi_Setpoint_Ankle = 0;
            left_leg->first_step = 1;
            left_leg->activate_in_3_steps = 1;
            left_leg->coef_in_3_steps = 0;
            left_leg->start_step = 0;

            jitter_flag = true;
          }

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

          // Once a new stride is identified (transition from State 1 > State 3) refresh biofeedback + update target score (if necessary)  
          if (FLAG_BIOFEEDBACK) {  
            if (leg->biofeedback_first_step) { // if biofeedback is turned on and this is the first step with biofeedback, we don't want to refresh the biofeedback system (ensures that the first stride that is used is a full stride with biofeedback on)
              leg->biofeedback_high_val = 0; // refresh the current high value
              leg->biofeedback_first_step = false; // indicate that the first step with biofeedback has been taken 
            } 
            else if (AUTOADJUST_BIOFEEDBACK && (leg->whos == biofeedbackLeg)){
              if(leg->whos == 'R') {
                refresh_countR++;
              } else if(leg->whos == 'L') {
                refresh_countL++;
              } 
            }
          }
         

          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
          leg->Max_FSR_Ratio = 0; 
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
      //leg->PID_Setpoint = leg->New_PID_Setpoint;
    }
    else {
      // Create the smoothed reference and call the PID
      //PID_Sigm_Curve(leg);
    }
  }



  if (Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4 || Control_Mode == 6) { //GO 5/19/19
    
    if (leg->state == 1) {
      
      double Baseline_Multiplier = 1;

      if (leg->Max_FSR_Ratio >= 1) { //Fixes an issue where the swing_counter and the DF spike during early swing would be scaled down by FSR ratio < Baseline
        Baseline_Multiplier = leg->Max_FSR_Ratio;
      }
      
      if (leg->swing_counter <= swing_counter_th*Baseline_Multiplier) { //Max FSR Ratio scales DF peak duration for speed adaptation
        leg->PID_Setpoint = 2*leg->New_PID_Setpoint*Baseline_Multiplier; //Activate Dorsiflexion during state 1. Max FSR Ratio scales DF peak for speed adaptation
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
      //leg->PID_Setpoint = leg->New_PID_Setpoint;
    }
    else {
      // Create the smoothed reference and call the PID
      //PID_Sigm_Curve(leg);
    }
  }
}// end function
