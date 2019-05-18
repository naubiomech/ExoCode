
//--------------------NEW FUNCTIONS-----------------
// Before state 3 in case of balance control was activated just in case of not negligible value
// of the force on the toe i.e. if you just walk on the heel you never feel the balance.
// Right now the state 3 is activate also in case of heel contact and the state 1 is activated
// in case of no contact of both the sensors
void state_machine(Leg* leg)
{
  // TN 5/8/19
  if (FLAG_TOE_HEEL_SENSORS ) {
    State_Machine_Toe_Heel_Sensors(leg);
  } else if (FLAG_TOE_SENSOR) {
    State_Machine_Toe_Sensor(leg);
  }

  if (FLAG_BALANCE) {
    State_Machine_Heel_Toe_Sensors_Balance(leg);
  } else {
    if (FLAG_BIOFEEDBACK) {
      State_Machine_Heel_Toe_Sensors_BioFeedback(leg);
    } else {
      State_Machine_Heel_Toe_Sensors(leg);
    }
  }
}




//-----------------------------------------------------------------------------------------------------
void State_Machine_Toe_Heel_Sensors(Leg * leg) {  // TN 5/8/19
  switch (leg->state)
  {
    case 1: //Swing
      // This flag enables the "set to zero" procedure for the left ankle.
      // When you're for at least 3 seconds in the same state, the torque reference is set to zero
      if (leg->set_2_zero == 1) {
        leg->set_2_zero = 0;
        leg->One_time_set_2_zero = 1;
      }
      else if ((leg->p_steps->curr_voltage > leg->fsr_percent_thresh_Toe * leg->fsr_Combined_peak_ref)&& (leg->p_steps->curr_voltage > leg->fsr_percent_thresh_Heel * leg->fsr_Combined_peak_ref))
      {
        leg->state_count_13++;
        // if you're in the same state for more than state_counter_th it means that it is not noise
        if (leg->state_count_13 >= state_counter_th)
        {
          leg->sigm_done = true;
          leg->Old_PID_Setpoint = leg->PID_Setpoint;
          leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee;  // TN 5/13/19

          if (leg->Previous_Setpoint_Ankle <= leg->Setpoint_Ankle)  // TN 5/9/19
            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle) * leg->coef_in_3_steps;
          else
            leg->New_PID_Setpoint = leg->Previous_Setpoint_Ankle - (leg->Previous_Setpoint_Ankle - leg->Setpoint_Ankle) * leg->coef_in_3_steps;
          // TN 5/9/19
          if (leg->Previous_Setpoint_Knee <= leg->Setpoint_Knee)
            leg->New_PID_Setpoint_Knee = leg->Previous_Setpoint_Knee + (leg->Setpoint_Knee - leg->Previous_Setpoint_Knee) * leg->coef_in_3_steps;
          else
            leg->New_PID_Setpoint_Knee = leg->Previous_Setpoint_Knee - (leg->Previous_Setpoint_Knee - leg->Setpoint_Knee) * leg->coef_in_3_steps;



          if (Flag_HLO && (leg->Previous_T_Opt <= leg->T_Opt)) {

            leg->T_Opt_Setpoint = leg->Previous_T_Opt + (leg->T_Opt - leg->Previous_T_Opt) * leg->coef_in_3_steps;
            Serial.print("Previous T Opt");
            Serial.print(leg->Previous_T_Opt);
            Serial.print("Current T Opt");
            Serial.println(leg->T_Opt_Setpoint);

          } else if (Flag_HLO && (leg->Previous_T_Opt > leg->T_Opt)) {

            leg->T_Opt_Setpoint = leg->Previous_T_Opt - (leg->Previous_T_Opt - leg->T_Opt) * leg->coef_in_3_steps;
            Serial.print("Previous T Opt");
            Serial.print(leg->Previous_T_Opt);
            Serial.print("Current T Opt");
            Serial.println(leg->T_Opt_Setpoint);

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
        leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee; // TN 5/9/19
        leg->state_old = leg->state;
        leg->New_PID_Setpoint = 0;
        leg->New_PID_Setpoint_Knee = 0; // TN 5/9/19
        leg->One_time_set_2_zero = 0;
        leg->Previous_Setpoint_Ankle = 0;
        leg->Previous_Setpoint_Knee = 0;  // TN 5/8/19
        leg->PID_Setpoint = 0;
        leg->PID_Setpoint_Knee = 0;  // TN 5/9/19
        leg->Setpoint_Ankle_Pctrl = 0;
        leg->Setpoint_Knee_Pctrl = 0;
        leg->Previous_Setpoint_Ankle_Pctrl = 0; //GO 4/21/19
        leg->Previous_Setpoint_Knee_Pctrl = 0; // TN 5/9/19

      }

      if ((leg->p_steps->curr_voltage < (leg->fsr_percent_thresh_Toe * leg->fsr_Combined_peak_ref)) && (leg->p_steps->curr_voltage < (leg->fsr_percent_thresh_Heel * leg->fsr_Combined_peak_ref))) // TN 5/15/19
      {
        leg->state_count_31++;
        if (leg->state_count_31 >= state_counter_th)
        {
          leg->sigm_done = true;
          leg->Old_PID_Setpoint = leg->PID_Setpoint;
          leg->Old_PID_Setpoint_Knee = leg->PID_Setpoint_Knee;  // TN 5/13/19
          leg->state_old = leg->state;
          //          leg->New_PID_Setpoint = 0 * leg->coef_in_3_steps;

          if (leg->Previous_Dorsi_Setpoint_Ankle <= leg->Dorsi_Setpoint_Ankle)
            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle + (leg->Dorsi_Setpoint_Ankle - leg->Previous_Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;
          else
            leg->New_PID_Setpoint = leg->Previous_Dorsi_Setpoint_Ankle - (leg->Previous_Dorsi_Setpoint_Ankle - leg->Dorsi_Setpoint_Ankle) * leg->coef_in_3_steps;

          // TN 5/9/19
          if (leg->Previous_Dorsi_Setpoint_Knee <= leg->Dorsi_Setpoint_Knee)
            leg->New_PID_Setpoint_Knee = leg->Previous_Dorsi_Setpoint_Knee + (leg->Dorsi_Setpoint_Knee - leg->Previous_Dorsi_Setpoint_Knee) * leg->coef_in_3_steps;
          else
            leg->New_PID_Setpoint_Knee = leg->Previous_Dorsi_Setpoint_Knee - (leg->Previous_Dorsi_Setpoint_Knee - leg->Dorsi_Setpoint_Knee) * leg->coef_in_3_steps;



          if (leg->New_PID_Setpoint == 0) { //GO 4/22/19
            leg->Previous_Dorsi_Setpoint_Ankle = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
          }
          if (leg->New_PID_Setpoint_Knee == 0) { // TN 5/13/19
            leg->Previous_Dorsi_Setpoint_Knee = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
          }
          leg->state = 1;
          leg->state_count_31 = 0;
          leg->state_count_13 = 0;
        }
      }
  }//end switch
  // Adjust the torque reference as a function of the step

  ref_step_adj(leg);
  //
//  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4) && leg->state == 3) { //GO 4/21/19
//    //    if (abs(leg->Previous_Setpoint_Ankle_Pctrl) <= abs(leg->Setpoint_Ankle)) {
//    //      leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl + (leg->Setpoint_Ankle - leg->Previous_Setpoint_Ankle_Pctrl) * leg->coef_in_3_steps; //Increment when the new setpoint is higher than the old
//    //    } else {
//    //      leg->p_steps->Setpoint = leg->Previous_Setpoint_Ankle_Pctrl - (leg->Previous_Setpoint_Ankle_Pctrl - leg->Setpoint_Ankle) * leg->coef_in_3_steps; //Decrement when the new setpoint is lower than the old
//    //    }
//    //    if (leg->p_steps->Setpoint == 0) {
//    //      leg->Previous_Setpoint_Ankle_Pctrl = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
//    //    }
//    //    // TN 5/9/19
//    //    if (abs(leg->Previous_Setpoint_Knee_Pctrl) <= abs(leg->Setpoint_Knee)) {
//    //      leg->p_steps->Setpoint_K = leg->Previous_Setpoint_Knee_Pctrl + (leg->Setpoint_Knee - leg->Previous_Setpoint_Knee_Pctrl) * leg->coef_in_3_steps; //Increment when the new setpoint is higher than the old
//    //    } else {
//    //      leg->p_steps->Setpoint_K = leg->Previous_Setpoint_Knee_Pctrl - (leg->Previous_Setpoint_Knee_Pctrl - leg->Setpoint_Knee) * leg->coef_in_3_steps; //Decrement when the new setpoint is lower than the old
//    //    }
//    //    if (leg->p_steps->Setpoint_K == 0) {
//    //      leg->Previous_Setpoint_Knee_Pctrl = 0; //To avoid an issue where after reaching ZT, stopping walking, and restarting walking the torque decrements from the previous down to ZT again
//    //    }
//    leg->PID_Setpoint = leg->Setpoint_Ankle_Pctrl * leg->coef_in_3_steps;
//    leg->PID_Setpoint_Knee = leg->Setpoint_Knee_Pctrl * leg->coef_in_3_steps;
//    //    Serial.print("coef_in_3_steps");
//    //    Serial.print(leg->coef_in_3_steps);
//  }
//  else {
//
//    if (N1 < 1 || N2 < 1 || N3 < 1) {
//      leg->PID_Setpoint = leg->New_PID_Setpoint;
//      leg->PID_Setpoint_Knee = leg->New_PID_Setpoint_Knee;  // TN 5/9/19
//    }
//    else {
//      // Create the smoothed reference and call the PID
//      PID_Sigm_Curve(leg);
//      PID_Sigm_Curve_Knee(leg);
//    }
//
//  }

//  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4) && leg->state == 1) {
//    leg->Setpoint_Ankle_Pctrl =  leg->New_PID_Setpoint;  // TN 5/9/19   need to check
//    leg->Setpoint_Knee_Pctrl =  leg->New_PID_Setpoint_Knee;  // TN 5/9/19  need to check
//  }
//  else {
//
//    if (N1 < 1 || N2 < 1 || N3 < 1) {
//      leg->PID_Setpoint = leg->New_PID_Setpoint;
//      leg->PID_Setpoint_Knee = leg->New_PID_Setpoint_Knee;  // TN 5/9/19
//    }
//    else {
//      // Create the smoothed reference and call the PID
//      PID_Sigm_Curve(leg);
//      PID_Sigm_Curve_Knee(leg);
//    }
//
//  }

    if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4) && leg->state == 3) {
      leg->PID_Setpoint = leg->Setpoint_Ankle_Pctrl * leg->coef_in_3_steps;
      leg->PID_Setpoint_Knee = leg->Setpoint_Knee_Pctrl * leg->coef_in_3_steps;
      Serial.print("coef_in_3_steps");
      Serial.print(leg->coef_in_3_steps);
    }
    else {
  
      if (N1 < 1 || N2 < 1 || N3 < 1) {
        leg->PID_Setpoint = leg->New_PID_Setpoint;
        leg->PID_Setpoint_Knee = leg->New_PID_Setpoint_Knee;
      }
      else {
        // Create the smoothed reference and call the PID
        PID_Sigm_Curve(leg);
        
      }
  
    }
  
    if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4) && leg->state == 1) {
      leg->PID_Setpoint = 0;
      leg->PID_Setpoint_Knee = 0;
    }
    else {
  
      if (N1 < 1 || N2 < 1 || N3 < 1) {
        leg->PID_Setpoint = leg->New_PID_Setpoint;
        leg->PID_Setpoint_Knee = leg->New_PID_Setpoint_Knee;
      }
      else {
        // Create the smoothed reference and call the PID
        PID_Sigm_Curve(leg);
      }
  
    }

}// end function

// TN 5/8/19


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
            Serial.print("Previous T Opt");
            Serial.print(leg->Previous_T_Opt);
            Serial.print("Current T Opt");
            Serial.println(leg->T_Opt_Setpoint);

          } else if (Flag_HLO && (leg->Previous_T_Opt > leg->T_Opt)) {

            leg->T_Opt_Setpoint = leg->Previous_T_Opt - (leg->Previous_T_Opt - leg->T_Opt) * leg->coef_in_3_steps;
            Serial.print("Previous T Opt");
            Serial.print(leg->Previous_T_Opt);
            Serial.print("Current T Opt");
            Serial.println(leg->T_Opt_Setpoint);

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
          //          leg->New_PID_Setpoint = 0 * leg->coef_in_3_steps;


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
  }//end switch
  // Adjust the torque reference as a function of the step
  ref_step_adj(leg);

  if ((Control_Mode == 2 || Control_Mode == 3 || Control_Mode == 4) && leg->state == 3) {
    leg->PID_Setpoint = leg->Setpoint_Ankle_Pctrl * leg->coef_in_3_steps;
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
            Serial.print("Previous T Opt");
            Serial.print(leg->Previous_T_Opt);
            Serial.print("Current T Opt");
            Serial.println(leg->T_Opt_Setpoint);

          } else if (Flag_HLO && (leg->Previous_T_Opt > leg->T_Opt)) {

            leg->T_Opt_Setpoint = leg->Previous_T_Opt - (leg->Previous_T_Opt - leg->T_Opt) * leg->coef_in_3_steps;
            Serial.print("Previous T Opt");
            Serial.print(leg->Previous_T_Opt);
            Serial.print("Current T Opt");
            Serial.println(leg->T_Opt_Setpoint);

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
