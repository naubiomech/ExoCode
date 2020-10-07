
//--------------------NEW FUNCTIONS-----------------
// Before state 3 in case of balance control was activated just in case of not negligible value
// of the force on the toe i.e. if you just walk on the heel you never feel the balance.
// Right now the state 3 is activate also in case of heel contact and the state 1 is activated
// in case of no contact of both the sensors
void state_machine(Leg* leg)
{
  // TN 5/8/19
  if (FLAG_TOE_HEEL_SENSORS &&  Control_Mode != 100) {
    State_Machine_Toe_Heel_Sensors(leg);
  }else if(Control_Mode == 100){
    State_BangBang(leg);
  } else if (FLAG_TOE_SENSOR) {
    State_Machine_Toe_Sensor(leg);
  }else if (FLAG_BALANCE) {
    State_Machine_Heel_Toe_Sensors_Balance(leg);
  } else {
    if (FLAG_BIOFEEDBACK) {
      State_Machine_Heel_Toe_Sensors_BioFeedback(leg);
    } else {
      State_BangBang(leg);
    }
  }
}




//-----------------------------------------------------------------------------------------------------
void State_Machine_Toe_Heel_Sensors(Leg * leg) {  
  switch (leg->state)
  {
    case 1: //Swing
      // This flag enables the "set to zero" procedure for the left ankle.
      // When you're for at least 3 seconds in the same state, the torque reference is set to zero
      if (leg->set_2_zero == 1) {
        leg->set_2_zero = 0;
        leg->One_time_set_2_zero = 1;
      }
      else if (leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref ||  leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) 
      {
        leg->state_count_13++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_13 >= state_counter_th)
        {
          if (Control_Mode == 100) { //Increment set point for bang-bang GO - 5/19/19
            leg->sigm_done = true;
            leg->sigm_done_Knee = true; // SS 1/21/2020
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee;// SS 1/15/2020


            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;
            leg->New_PID_Setpoint_Knee = leg->Previous_Setpoint_Knee + (leg->Setpoint_Knee - leg->Previous_Setpoint_Knee) * leg->coef_in_3_steps_Knee;// SS 1/15/2020



            if (Flag_HLO)

              leg->T_Opt_Setpoint = leg->Previous_T_Opt + (leg->T_Opt - leg->Previous_T_Opt) * leg->coef_in_3_steps_Ankle;

          }

          else if (Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4) { //Increment the max set point for proportional control GO - 5/19/19

            leg->p_steps->Setpoint_A = leg->Previous_Setpoint_Ankle_Pctrl + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle_Pctrl) * leg->coef_in_3_steps_Ankle; //Increment when the new setpoint is higher than the old

            if (leg->p_steps->Setpoint_A == 0) {
              leg->Previous_Setpoint_Ankle_Pctrl = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
            }
            
            leg->p_steps->Setpoint_K = leg->Previous_Setpoint_Knee_Pctrl + (leg->Setpoint_Knee - leg->Previous_Setpoint_Knee_Pctrl) * leg->coef_in_3_steps_Knee; //Increment when the new setpoint is higher than the old

            if (leg->p_steps->Setpoint_K == 0) {// SS 1/15/2020
              leg->Previous_Setpoint_Knee_Pctrl = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
            }// SS 1/15/2020

          }
          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
//          leg->state_count_32 = 0;  // SS 9/10/2019
//          leg->state_count_23 = 0;  // SS 9/10/2019
        }
      }
      // SS 9/10/2019
//      else if ((leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref && leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)) { //Heel active but not toe
//        if (Control_Mode != 100) {
//          leg->state_count_12++;
//          if (leg->state_count_12 >= state_counter_th) {
//            if (Control_Mode == 4) { //Increment the max set point for proportional control GO - 5/19/19
//
//
//
//              leg->p_steps->Setpoint_K = leg->Previous_Setpoint_Knee_Pctrl + (leg->Setpoint_Knee - leg->Previous_Setpoint_Knee_Pctrl) * leg->coef_in_3_steps_Knee; //Increment when the new setpoint is higher than the old
//
//              if (leg->p_steps->Setpoint_K == 0) {
//                leg->Previous_Setpoint_Knee_Pctrl = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
//              }
//            }
//
//
//            leg->state_old = leg->state;
//            leg->state = 2;
//            leg->state_count_23 = 0;
//            leg->state_count_32 = 0;
//            leg->state_count_21 = 0;
//            leg->state_count_12 = 0;
//          }
//        }
//      }
      break;

//    case 2: //Early Stance //GO 5/19/19 // SS 9/10/2019
//      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
//        leg->sigm_done = true;
//        leg->sigm_done_Knee = true;   // TN 5/20/19
//        leg->Old_PID_Setpoint = leg->PID_Setpoint;
//        leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee;
//        leg->state_old = leg->state;
//        leg->New_PID_Setpoint = 0;
//        leg->New_PID_Setpoint_Knee = 0;
//        leg->One_time_set_2_zero = 0;
//        leg->Previous_Setpoint_Ankle = 0;
//        leg->Previous_Setpoint_Knee = 0;
//        leg->PID_Setpoint = 0;
//        leg->PID_Setpoint_Knee = 0;
//        leg->Setpoint_Ankle_Pctrl = 0;
//        leg->Setpoint_Knee_Pctrl = 0;
//        leg->Previous_Setpoint_Ankle_Pctrl = 0;
//        leg->Previous_Setpoint_Knee_Pctrl = 0;
//      }
//      else if ((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)) {
//        leg->state_count_23++;
//        if (leg->state_count_23 >= state_counter_th) {
//
//          if (Control_Mode == 100) { //Increment set point for bang-bang GO - 5/19/19
//            leg->sigm_done = true;
//            leg->Old_PID_Setpoint = leg->PID_Setpoint;
//
//            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;
//
//
//
//            if (Flag_HLO)
//
//              leg->T_Opt_Setpoint = leg->Previous_T_Opt + (leg->T_Opt - leg->Previous_T_Opt) * leg->coef_in_3_steps_Ankle;
//
//          }
//
//          else if (Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4) { //Increment the max set point for proportional control GO - 5/19/19
//
//            leg->p_steps->Setpoint_A = leg->Previous_Setpoint_Ankle_Pctrl + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle_Pctrl) * leg->coef_in_3_steps_Ankle; //Increment when the new setpoint is higher than the old
//
//            if (leg->p_steps->Setpoint_A == 0) {
//              leg->Previous_Setpoint_Ankle_Pctrl = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
//            }
//            // TN 5/20/19
//
//          }
//
//          leg->state_old = leg->state;
//          leg->state = 3;
//          leg->state_count_23 = 0;
//          leg->state_count_32 = 0;
//          leg->state_count_31 = 0;
//          leg->state_count_13 = 0;
//        }
//      }
//      else if ((leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref && leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)) {
//        leg->state_count_21++;
//        if (leg->state_count_21 >= 4 * state_counter_th) {
//          if (Control_Mode == 100) {
//            leg->sigm_done = true;
//            leg->Old_PID_Setpoint = leg->PID_Setpoint;
//          }
//
//
//          leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;
//
//
//
//
//
//          leg->New_PID_Setpoint_Knee = leg->Previous_Flexion_Setpoint_Knee + (leg->Flexion_Setpoint_Knee - leg->Previous_Flexion_Setpoint_Knee) * leg->coef_in_3_steps_Knee;
//
//
//
//          if (leg->New_PID_Setpoint == 0) { //GO 4/22/19
//            leg->Previous_Dorsi_Setpoint_Ankle = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
//          }
//
//          if (leg->New_PID_Setpoint_Knee == 0) { //TN 5/22/19
//            leg->Previous_Flexion_Setpoint_Knee = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
//          }
//
//
//          leg->state_old = leg->state;
//          leg->state = 1;
//          leg->state_count_21 = 0;
//          leg->state_count_12 = 0;
//          leg->state_count_13 = 0;
//          leg->state_count_31 = 0;
//        }
//      }
//
//      break;

    case 3: //Late Stance
      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true;
        leg->sigm_done_Knee = true;  // TN 5/20/19
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee; // TN 5/9/19
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->New_PID_Setpoint_Knee = 0; // TN 5/9/19
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->Previous_Setpoint_Knee = 0;  // TN 5/8/19
        leg->PID_Setpoint = 0;
//        leg->Angular_Impulse = 0; // SS 6/8/2020
        leg->PID_Setpoint_Knee = 0;
//        leg->Angular_Impulse_Knee = 0; // SS 6/8/2020 
        leg->Setpoint_Ankle_Pctrl = 0;
        leg->Setpoint_Knee_Pctrl = 0;
        leg->Previous_Setpoint_Ankle_Pctrl = 0; //GO 4/21/19
        leg->Previous_Setpoint_Knee_Pctrl = 0; 

      }

      else if ((leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref))
      {
        leg->state_count_31++;
        if (leg->state_count_31 >= state_counter_th)
        {
          if (Control_Mode == 100) {
            leg->sigm_done = true;
            leg->sigm_done_Knee = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee;  // TN 5/13/19
          }


          leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;
          leg->New_PID_Setpoint_Knee = leg->Previous_Flexion_Setpoint_Knee + (leg->Flexion_Setpoint_Knee - leg->Previous_Flexion_Setpoint_Knee) * leg->coef_in_3_steps_Knee;


          if (leg->New_PID_Setpoint == 0) { //GO 4/22/19
            leg->Previous_Dorsi_Setpoint_Ankle = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
          }

          if (leg->New_PID_Setpoint_Knee == 0) { // TN 5/13/19
            leg->Previous_Flexion_Setpoint_Knee = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
          }

          leg->state_old = leg->state;
          leg->state = 1;
          leg->state_count_31 = 0;
          leg->state_count_13 = 0;
//          leg->state_count_12 = 0;  // SS 9/10/2019
//          leg->state_count_21 = 0;  // SS 9/10/2019


        }
      }
// SS 9/10/2019
//      else if ((leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) && (leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)) {
//        if (Control_Mode != 100) {
//          leg->state_count_32++;
//          if (leg->state_count_32 >= state_counter_th) {
//
//            if (Control_Mode == 4) { //Increment the max set point for proportional control GO - 5/19/19
//
//              leg->p_steps->Setpoint_K = leg->Previous_Setpoint_Knee_Pctrl + (leg->Setpoint_Knee - leg->Previous_Setpoint_Knee_Pctrl) * leg->coef_in_3_steps_Knee; //Increment when the new setpoint is higher than the old
//
//              if (leg->p_steps->Setpoint_K == 0) {
//                leg->Previous_Setpoint_Knee_Pctrl = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
//              }
//            }
//
//
//            leg->state_old = leg->state;
//            leg->state = 2;
//            leg->state_count_32 = 0;
//            leg->state_count_23 = 0;  
//            leg->state_count_12 = 0;
//            leg->state_count_21 = 0;
//          }
//        }
//      }

  }//end switch
  // Adjust the torque reference as a function of the step

  ref_step_adj(leg);

  if (Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4) {  // TN 5/20/19 set the knee control in state 2 and 3
    if (leg->state == 3) {
      leg->PID_Setpoint = leg->Setpoint_Ankle_Pctrl; // * leg->coef_in_3_steps;
      leg->PID_Setpoint_Knee = leg->Setpoint_Knee_Pctrl;  // SS 9/10/2019
    }
//    if (leg->state == 2) {
//      leg->PID_Setpoint_Knee = leg->Setpoint_Knee_Pctrl; // * leg->coef_in_3_steps;
//      leg->PID_Setpoint = 0;
//    }
    if (leg->state == 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
      leg->PID_Setpoint_Knee = leg->New_PID_Setpoint_Knee;// SS 9/10/2019

    }
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
      leg->PID_Setpoint_Knee = leg->New_PID_Setpoint_Knee;// SS 9/10/2019
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
      PID_Sigm_Curve_Knee(leg);// SS 9/10/2019

    }



  }

}// end function


///-----------------------------------------------

void State_Machine_Toe_Sensor(Leg * leg) {
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
          leg->sigm_done_Knee = true;
          leg->Old_PID_Setpoint = leg->PID_Setpoint;
          leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee;// SS 1/9/2020

          if (abs(leg->Dorsi_Setpoint_Ankle) > 0) {
            leg->Old_PID_Setpoint = 0;
          } else {
            leg->Previous_Dorsi_Setpoint_Ankle = 0;
          }

          if (abs(leg->Flexion_Setpoint_Knee) > 0) {
            leg->Old_PID_Setpoint_Knee = 0;// SS 1/9/2020
          } else {
            leg->Previous_Flexion_Setpoint_Knee = 0;// SS 1/9/2020
          }

          if (leg->Previous_Setpoint_Ankle <= leg->Setpoint_Ankle) {
            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;
          } else {
            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle - (leg->Previous_Setpoint_Ankle - leg->Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;
          }

          if (leg->Previous_Setpoint_Knee <= leg->Setpoint_Knee) {// SS 1/9/2020
            leg->New_PID_Setpoint_Knee = leg->Previous_Setpoint_Knee + (leg->Setpoint_Knee - leg->Previous_Setpoint_Knee) * leg->coef_in_3_steps_Knee;
          } else {
            leg->New_PID_Setpoint_Knee = leg->Previous_Setpoint_Knee - (leg->Previous_Setpoint_Knee - leg->Setpoint_Knee) * leg->coef_in_3_steps_Knee;
          }// SS 1/9/2020
          

          if (Flag_HLO && (leg->Previous_T_Opt <= leg->T_Opt)) {

            leg->T_Opt_Setpoint = leg->Previous_T_Opt + (leg->T_Opt - leg->Previous_T_Opt) * leg->coef_in_3_steps_Ankle;

          } else if (Flag_HLO && (leg->Previous_T_Opt > leg->T_Opt)) {

            leg->T_Opt_Setpoint = leg->Previous_T_Opt - (leg->Previous_T_Opt - leg->T_Opt) * leg->coef_in_3_steps_Ankle;

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
        leg->sigm_done_Knee = true;// SS 1/21/2020
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee;// SS 1/9/2020
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->New_PID_Setpoint_Knee = 0;// SS 1/9/2020
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->PID_Setpoint = 0;
//        leg->Angular_Impulse = 0; // SS 6/8/2020
        leg->Setpoint_Ankle_Pctrl = 0;
        leg->Previous_Setpoint_Knee = 0;// SS 1/9/2020
        leg->PID_Setpoint_Knee = 0;// SS 1/9/2020
//        leg->Angular_Impulse_Knee = 0; // SS 6/8/2020
        leg->Setpoint_Knee_Pctrl = 0;// SS 1/9/2020
      }

      if ((leg->p_steps->curr_voltage < (leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)))
      {
        leg->state_count_31++;
        if (leg->state_count_31 >= state_counter_th)
        {
          leg->sigm_done = true;
          leg->sigm_done_Knee = true;// SS 1/21/2020
          leg->Old_PID_Setpoint = leg->PID_Setpoint;
          leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee;// SS 1/9/2020
          leg->state_old = leg->state;

          if (leg->Previous_Dorsi_Setpoint_Ankle <= leg->Dorsi_Setpoint_Ankle) {
            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;
          } else {
            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle - (leg->Previous_Dorsi_Setpoint_Ankle - leg->Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;
          }
          
          if (leg->Previous_Flexion_Setpoint_Knee <= leg->Flexion_Setpoint_Knee) {// SS 1/9/2020
            leg->New_PID_Setpoint_Knee = leg->Previous_Flexion_Setpoint_Knee + (leg->Flexion_Setpoint_Knee - leg->Previous_Flexion_Setpoint_Knee) * leg->coef_in_3_steps_Knee;
          } else {
            leg->New_PID_Setpoint_Knee = leg->Previous_Flexion_Setpoint_Knee - (leg->Previous_Flexion_Setpoint_Knee - leg->Flexion_Setpoint_Knee) * leg->coef_in_3_steps_Knee;
          }// SS 1/9/2020

          leg->state = 1;
          leg->state_count_31 = 0;
          leg->state_count_13 = 0;
        }
      }
  }// end switch
  // Adjust the torque reference as a function of the step
  ref_step_adj(leg);

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4) && leg->state == 3) {
    leg->PID_Setpoint = leg->Setpoint_Ankle_Pctrl;
    leg->PID_Setpoint_Knee = leg->Setpoint_Knee_Pctrl;// SS 1/9/2020
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
      leg->PID_Setpoint_Knee = leg->New_PID_Setpoint_Knee;// SS 1/9/2020
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
      PID_Sigm_Curve_Knee(leg);
    }

  }

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4) && leg->state == 1) {
    leg->PID_Setpoint = 0;
    leg->PID_Setpoint_Knee = 0;// SS 1/9/2020
//    leg->Angular_Impulse = 0; // SS 6/8/2020
//    leg->Angular_Impulse_Knee = 0; // SS 6/8/2020
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
      leg->PID_Setpoint_Knee = leg->New_PID_Setpoint_Knee; // SS 1/15/2020
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
      PID_Sigm_Curve_Knee(leg);// SS 1/9/2020
    }

  }
}// end function
//------------------------------------------------------------------
void State_BangBang(Leg * leg) {  // SS 1/27/2020
  switch (leg->state)
  {
    case 1: //Swing
      // This flag enables the "set to zero" procedure for the left ankle.
      // When you're for at least 3 seconds in the same state, the torque reference is set to zero
      if (leg->set_2_zero == 1) {
        leg->set_2_zero = 0;
        leg->One_time_set_2_zero = 1;
      }
      else if( (leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)  &&  (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) )// Transition from state 1 to state 2// SS 1/27/2020
      {
        leg->state_count_12++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_12 >= state_counter_th)
        {
          if (Control_Mode == 100) { //Increment set point for bang-bang GO - 5/19/19
            leg->sigm_done = true;
            leg->sigm_done_Knee = true; 
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee;

            leg->New_PID_Setpoint_Knee = leg->Previous_Setpoint_Knee + (leg->Setpoint_Knee - leg->Previous_Setpoint_Knee) * leg->coef_in_3_steps_Knee;

            if (Flag_HLO)
              leg->T_Opt_Setpoint = leg->Previous_T_Opt + (leg->T_Opt - leg->Previous_T_Opt) * leg->coef_in_3_steps_Ankle;

          }

          leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

          if (leg->New_PID_Setpoint == 0) { 
            leg->Previous_Dorsi_Setpoint_Ankle = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
          }

          leg->state_old = leg->state;
          leg->state = 2;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_23 = 0;
          leg->state_count_32 = 0;
          leg->state_count_31 = 0;
          leg->state_count_21 = 0;
        }
      }
      else if ((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) || ((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref))) // Transition from state 1 to state 2// SS 1/27/2020
      {
        leg->state_count_13++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_13 >= state_counter_th)
        {
          if (Control_Mode == 100) { //Increment set point for bang-bang GO - 5/19/19
            leg->sigm_done = true;
            leg->sigm_done_Knee = true; 
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee;


            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;
            leg->New_PID_Setpoint_Knee = leg->Previous_Setpoint_Knee + (leg->Setpoint_Knee - leg->Previous_Setpoint_Knee) * leg->coef_in_3_steps_Knee;

            if (Flag_HLO)
              leg->T_Opt_Setpoint = leg->Previous_T_Opt + (leg->T_Opt - leg->Previous_T_Opt) * leg->coef_in_3_steps_Ankle;

          }
          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_23 = 0;
          leg->state_count_32 = 0;
          leg->state_count_31 = 0;
          leg->state_count_21 = 0;
        }
      }
      break;

    case 2: //Early Stance // SS 1/27/2020
      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true;
        leg->sigm_done_Knee = true; 
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee;
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->New_PID_Setpoint_Knee = 0;
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->Previous_Setpoint_Knee = 0;
        leg->PID_Setpoint = 0;
//        leg->Angular_Impulse = 0; // SS 6/8/2020
        leg->PID_Setpoint_Knee = 0;
//        leg->Angular_Impulse_Knee = 0; // SS 6/8/2020
        leg->Setpoint_Ankle_Pctrl = 0;
        leg->Setpoint_Knee_Pctrl = 0;
        leg->Previous_Setpoint_Ankle_Pctrl = 0;
        leg->Previous_Setpoint_Knee_Pctrl = 0;
      }
      else if ((leg->p_steps->curr_voltage_Toe > leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)) {// Transition from state 2 to state 3// SS 1/27/2020
        leg->state_count_23++;
        if (leg->state_count_23 >= state_counter_th) {

          if (Control_Mode == 100) { //Increment set point for bang-bang GO - 5/19/19
            leg->sigm_done = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;

            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

            if (Flag_HLO)
              leg->T_Opt_Setpoint = leg->Previous_T_Opt + (leg->T_Opt - leg->Previous_T_Opt) * leg->coef_in_3_steps_Ankle;

          }
          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_23 = 0;
          leg->state_count_31 = 0;
          leg->state_count_21 = 0;
        }
      }
      else if ((leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref && leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)) {// Transition from state 2 to state 1// SS 1/27/2020
        leg->state_count_21++;
        if (leg->state_count_21 >= 4 * state_counter_th) {
          if (Control_Mode == 100) {
            leg->sigm_done = true;
            leg->sigm_done_Knee = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee;
          }


          leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;
          leg->New_PID_Setpoint_Knee = leg->Previous_Flexion_Setpoint_Knee + (leg->Flexion_Setpoint_Knee - leg->Previous_Flexion_Setpoint_Knee) * leg->coef_in_3_steps_Knee;



          if (leg->New_PID_Setpoint == 0) { 
            leg->Previous_Dorsi_Setpoint_Ankle = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
          }
          if (leg->New_PID_Setpoint_Knee == 0) {
            leg->Previous_Flexion_Setpoint_Knee = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
          }


          leg->state_old = leg->state;
          leg->state = 1;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_23 = 0;
          leg->state_count_32 = 0;
          leg->state_count_31 = 0;
          leg->state_count_21 = 0;
        }
      }

      break;

    case 3: //Late Stance
      if ((leg->set_2_zero == 1) && (leg->One_time_set_2_zero)) {
        leg->sigm_done = true;
        leg->sigm_done_Knee = true; 
        leg->Old_PID_Setpoint = leg->PID_Setpoint;
        leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee;
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->New_PID_Setpoint_Knee = 0;
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->Previous_Setpoint_Knee = 0;
        leg->PID_Setpoint = 0;
//        leg->Angular_Impulse = 0; // SS 6/8/2020
        leg->PID_Setpoint_Knee = 0;  
//        leg->Angular_Impulse_Knee = 0; // SS 6/8/2020
        leg->Setpoint_Ankle_Pctrl = 0;
        leg->Setpoint_Knee_Pctrl = 0;
        leg->Previous_Setpoint_Ankle_Pctrl = 0; 
        leg->Previous_Setpoint_Knee_Pctrl = 0; 

      }

      else if ((leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->p_steps->curr_voltage_Heel <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref))
      {
        leg->state_count_31++;
        if (leg->state_count_31 >= state_counter_th)
        {
          if (Control_Mode == 100) {
            leg->sigm_done = true;
            leg->sigm_done_Knee = true;
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee; 
          }


          leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;
          leg->New_PID_Setpoint_Knee = leg->Previous_Flexion_Setpoint_Knee + (leg->Flexion_Setpoint_Knee - leg->Previous_Flexion_Setpoint_Knee) * leg->coef_in_3_steps_Knee;


          if (leg->New_PID_Setpoint == 0) {
            leg->Previous_Dorsi_Setpoint_Ankle = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
          }

          if (leg->New_PID_Setpoint_Knee == 0) {
            leg->Previous_Flexion_Setpoint_Knee = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
          }

          leg->state_old = leg->state;
          leg->state = 1;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_23 = 0;
          leg->state_count_32 = 0;
          leg->state_count_31 = 0;
          leg->state_count_21 = 0;


        }
      }
      else if( (leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)  &&  (leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) )// Transition from state 1 to state 2// SS 1/27/2020
      {
        leg->state_count_32++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_12 >= state_counter_th)
        {
          if (Control_Mode == 100) { //Increment set point for bang-bang GO - 5/19/19
            leg->sigm_done = true;
            leg->sigm_done_Knee = true; 
            leg->Old_PID_Setpoint = leg->PID_Setpoint;
            leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee;

            leg->New_PID_Setpoint_Knee = leg->Previous_Setpoint_Knee + (leg->Setpoint_Knee - leg->Previous_Setpoint_Knee) * leg->coef_in_3_steps_Knee;

            if (Flag_HLO)
              leg->T_Opt_Setpoint = leg->Previous_T_Opt + (leg->T_Opt - leg->Previous_T_Opt) * leg->coef_in_3_steps_Ankle;

          }

          leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

          if (leg->New_PID_Setpoint == 0) { 
            leg->Previous_Dorsi_Setpoint_Ankle = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
          }

          leg->state_old = leg->state;
          leg->state = 2;
          leg->state_count_12 = 0;
          leg->state_count_13 = 0;
          leg->state_count_23 = 0;
          leg->state_count_32 = 0;
          leg->state_count_31 = 0;
          leg->state_count_21 = 0;
        }
      }
// SS 9/10/2019
//      else if ((leg->p_steps->curr_voltage_Heel > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref) && (leg->p_steps->curr_voltage_Toe <= leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref)) {
//        if (Control_Mode != 100) {
//          leg->state_count_32++;
//          if (leg->state_count_32 >= state_counter_th) {
//
//            if (Control_Mode == 4) { //Increment the max set point for proportional control GO - 5/19/19
//
//              leg->p_steps->Setpoint_K = leg->Previous_Setpoint_Knee_Pctrl + (leg->Setpoint_Knee - leg->Previous_Setpoint_Knee_Pctrl) * leg->coef_in_3_steps_Knee; //Increment when the new setpoint is higher than the old
//
//              if (leg->p_steps->Setpoint_K == 0) {
//                leg->Previous_Setpoint_Knee_Pctrl = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
//              }
//            }
//
//
//            leg->state_old = leg->state;
//            leg->state = 2;
//            leg->state_count_32 = 0;
//            leg->state_count_23 = 0;  
//            leg->state_count_12 = 0;
//            leg->state_count_21 = 0;
//          }
//        }
//      }

  }//end switch
  // Adjust the torque reference as a function of the step

  ref_step_adj(leg);

  if (Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4) {  // TN 5/20/19 set the knee control in state 2 and 3
    if (leg->state == 3) {
      leg->PID_Setpoint = leg->Setpoint_Ankle_Pctrl; // * leg->coef_in_3_steps;
      leg->PID_Setpoint_Knee = leg->Setpoint_Knee_Pctrl;  // SS 9/10/2019
    }
//    if (leg->state == 2) {
//      leg->PID_Setpoint_Knee = leg->Setpoint_Knee_Pctrl; // * leg->coef_in_3_steps;
//      leg->PID_Setpoint = 0;
//    }
    if (leg->state == 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
      leg->PID_Setpoint_Knee = leg->New_PID_Setpoint_Knee;// SS 9/10/2019

    }
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
      leg->PID_Setpoint_Knee = leg->New_PID_Setpoint_Knee;// SS 9/10/2019
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
      PID_Sigm_Curve_Knee(leg);// SS 9/10/2019

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

            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle - (leg->Previous_Setpoint_Ankle - leg->Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

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

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle - (leg->Previous_Dorsi_Setpoint_Ankle - leg->Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

          }

          leg->state = 1;
          leg->state_count_31 = 0;
          leg->state_count_13 = 0;
        }
      }
  }// end switch
  // Adjust the torque reference as a function of the step
  ref_step_adj(leg);

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4) && leg->state == 3) {
    leg->PID_Setpoint = leg->Setpoint_Ankle_Pctrl;
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
      PID_Sigm_Curve_Knee(leg);
    }

  }

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4) && leg->state == 1) {
    leg->PID_Setpoint = 0;
  }
  else {

    if (N1 < 1 || N2 < 1 || N3 < 1) {
      leg->PID_Setpoint = leg->New_PID_Setpoint;
    }
    else {
      // Create the smoothed reference and call the PID
      PID_Sigm_Curve(leg);
      PID_Sigm_Curve_Knee(leg);
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

            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle - (leg->Previous_Setpoint_Ankle - leg->Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

          }

          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_count_13 = 0;
          leg->state_count_31 = 0;
        }
      }
      else if ((leg->FSR_Heel_Average > leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref)) {
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

            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle - (leg->Previous_Setpoint_Ankle - leg->Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

          }

          leg->state_old = leg->state;
          leg->state = 3;
          leg->state_count_23 = 0;
          leg->state_count_32 = 0;
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

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle - (leg->Previous_Dorsi_Setpoint_Ankle - leg->Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

          }

          leg->state = 1;
          //            leg->state_count_21 = 0;
          leg->state_count_12 = 0;

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

      if ((leg->FSR_Toe_Average <= (leg->fsr_percent_thresh_Toe * leg->fsr_Toe_peak_ref) && (leg->FSR_Heel_Average <= leg->fsr_percent_thresh_Heel * leg->fsr_Heel_peak_ref)))
      {
        leg->state_count_31++;
        if (leg->state_count_31 >= state_counter_th)
        {
          leg->sigm_done = true;
          leg->Old_PID_Setpoint = leg->PID_Setpoint;
          leg->state_old = leg->state;


          if (leg->Previous_Dorsi_Setpoint_Ankle <= leg->Dorsi_Setpoint_Ankle) {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle - (leg->Previous_Dorsi_Setpoint_Ankle - leg->Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

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

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

          } else {

            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle - (leg->Previous_Dorsi_Setpoint_Ankle - leg->Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps_Ankle;

          }

          leg->state = 2;
          leg->state_count_32 = 0;
          leg->state_count_23 = 0;
        }

      }//end if go to state 2

  }// end switch





  // Adjust the torque reference as a function of the step
  ref_step_adj(leg);

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4) && leg->state == 3) {
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

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4) && leg->state == 1) {
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
