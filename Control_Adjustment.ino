// in this file there are the functions that modify the control strategy both in the strategy and in the shaping factors (N3) when required.


// take baseline for some of the controls during the plantarflexion state , i.e. 3
int take_baseline(Leg* leg, int R_state_l, int R_state_old_l, steps* p_steps_l, int* p_flag_take_baseline_l) {

  // update the voltage peak


  if (p_steps_l->curr_voltage > p_steps_l->peak)
    p_steps_l->peak =  p_steps_l->curr_voltage;

  if (p_steps_l->flag_start_plant == false) // if it is true it means you started the step. Here I inizialize the parameters for speed adaption.
  {
    p_steps_l->plant_time = millis(); // start the plantarflexion
    p_steps_l->dorsi_time = millis() - (p_steps_l->dorsi_time); // calculate the dorsiflexion that has just finished

    if (p_steps_l->dorsi_time <= step_time_length / 4) // if <50ms probably it is noise
    {
      p_steps_l->peak = 0;
      p_steps_l->flag_start_plant = false;
      //        Serial.println(" BASE Dorsi too short");
      return 0;
    } else {
      p_steps_l->flag_start_plant = true; // Parameters inizialized Start a step
      //        Serial.println(" BASE Start Plantar");
    }
  }

  if (p_steps_l->flag_start_plant) {
    // If a step has started i.e. the states have passed from 1 or 2 to 3
    // if you transit from 3 to 1 plantar flexion is completed and start dorsiflexion

    if ((R_state_old_l == 3) && (R_state_l == 1 || R_state_l == 2)) {
      //      Serial.println(" BASE Start Dorsi");

      // start dorsiflexion

      p_steps_l->dorsi_time = millis();
      // calculate plantarflexion
      p_steps_l->plant_time = millis() - (p_steps_l->plant_time);

      if (p_steps_l->plant_time <= step_time_length)
      {
        p_steps_l->flag_start_plant = false;
        //        Serial.println("BASE Plant too short"); // it means is a glitch not a real step
        return 0;
      } else {
        
        p_steps_l->flag_start_plant = false; // you have provided one plant
        //        Serial.println("Increase Plant number");
        p_steps_l->count_plant_base++; // you have accomplished a step
        //        Serial.println(p_steps_l->count_plant_base);
      }

      //      Serial.print(" BASE Plant Time = ");
      //      Serial.println(p_steps_l->plant_time);

      if ((p_steps_l->count_plant_base) >= 2) { // avoid the first step just to be sure

        //        // this is the time window of the filter for the dorsiflexion
        if (((p_steps_l->count_plant_base) - 2) >= n_step_baseline) {

          p_steps_l->dorsi_mean = 0;
          p_steps_l->plant_mean = 0;
          p_steps_l->plant_peak_mean_temp = 0;


          for (int i = 0; i < n_step_baseline - 1; i++)
          {
            p_steps_l->four_step_dorsi_time[i] = p_steps_l->four_step_dorsi_time[i + 1];
            p_steps_l->dorsi_mean += p_steps_l->four_step_dorsi_time[i];

            p_steps_l->four_step_plant_time[i] = p_steps_l->four_step_plant_time[i + 1];
            p_steps_l->plant_mean += p_steps_l->four_step_plant_time[i];

            p_steps_l->four_step_plant_peak[i] = p_steps_l->four_step_plant_peak[i + 1];
            p_steps_l->plant_peak_mean_temp += p_steps_l->four_step_plant_peak[i];
          }

          p_steps_l->four_step_dorsi_time[n_step_baseline - 1] = p_steps_l->dorsi_time;
          p_steps_l->dorsi_mean += p_steps_l->dorsi_time;

          p_steps_l->four_step_plant_time[n_step_baseline - 1] = p_steps_l->plant_time;
          p_steps_l->plant_mean += p_steps_l->plant_time;

          p_steps_l->four_step_plant_peak[n_step_baseline - 1] = p_steps_l->peak;
          p_steps_l->plant_peak_mean_temp += p_steps_l->peak;

          p_steps_l->dorsi_mean = (p_steps_l->dorsi_mean) / n_step_baseline;
          p_steps_l->plant_mean = p_steps_l->plant_mean / n_step_baseline;
          p_steps_l->plant_peak_mean_temp = 1.0 * (p_steps_l->plant_peak_mean_temp) / n_step_baseline; //Changed from 0.9 to 1.0 by GO on 4/22/19

          //HERE

          //          Serial.println("BASE before return");
          //          Serial.print(" Peak ");
          //          Serial.println(p_steps_l->peak);
          //          Serial.print(" N ");
          //          Serial.println(p_steps_l->count_plant_base);
        }
        else {
          p_steps_l->four_step_dorsi_time[p_steps_l->count_plant_base - 2] = p_steps_l->dorsi_time;

          p_steps_l->four_step_plant_time[p_steps_l->count_plant_base - 2] = p_steps_l->plant_time;

          p_steps_l->four_step_plant_peak[p_steps_l->count_plant_base - 2] = p_steps_l->peak;
          //          Serial.println("Inside Peak vector ");

          for (int i = 0; i < n_step_baseline; i++) {
            //            Serial.println(p_steps_l->four_step_plant_peak[i]);
          }
        }

        if (((p_steps_l->count_plant_base) - 2) >= n_step_baseline) {
          //          Serial.print("BASE return peak mean temporary");
          //          Serial.println(p_steps_l->plant_peak_mean_temp);
        }
        if (((p_steps_l->count_plant_base) - 2) >= n_step_baseline) {
          (p_steps_l->count_plant_base) = 0;
          *p_flag_take_baseline_l = 0;
          leg->baseline_value = p_steps_l->plant_peak_mean;
          if (leg->whos == 'R') {baselineRight = true;}
          if (leg->whos == 'L') {baselineLeft = true;}
          if ((baselineRight) && (baselineLeft)) {
            update_biofeedback_baseline();
            send_command_message('n', emptyData, 1); //GO 4/23/19 to communicate that baseline is done, the array sent in position two has one position initialized as zero
            baselineRight = false;
            baselineLeft = false;
          }
          return (p_steps_l->count_plant_base);

        } // return 1 activate a flag that stops the calc of the baseline
      }// end if count_plant>2

    }//end dorsiflexion

  }// end start_step

  if (((R_state_l == 1) || (R_state_l == 2)) && R_state_old_l == 3)
    p_steps_l->peak = 0;



}// end take_baseline



//------------------------------------------------------------------------------


double Control_Adjustment(Leg* leg, int R_state_l, int R_state_old_l, steps* p_steps_l, double N3_l, double New_PID_Setpoint_l, double* p_Setpoint_Ankle_l, double * p_Setpoint_Ankle_Pctrl_l, int Control_Mode_l, double prop_gain_l, double taking_baseline_l, double *p_FSR_Ratio, double* p_Max_FSR_Ratio) {

  // Control Mode 2: Balance control
  // Control Mode 3: Joint Moment control, the torque is a percentage of the extimated Ankle moment. The mapping function that estimated the ankle moment use a ratio (p_FSR_Ratio) which depends on the current force of pressure
  // and the baseline value. The baseline value can be updated several times also during the execution of the task

  //otherwise Control Mode =  100 implies the classic bang bang whose shaping is based on N3
  //  Despite control mode 2 and 3 do not uses the N3 the function still returns a number associated to N3 which is not used.

  if (taking_baseline_l) { // if I am taking the baseline adapt some parameters for the controls

    //--------------------------------
    if ((R_state_l == 3) && (R_state_old_l == 1 || R_state_old_l == 2))
    {
      if (Control_Mode_l == 3) { // JOINT MOMENT CONTROL also known as pivot proportional control while taking the baseline
        // TN 7/15/19
        if (p_steps_l->plant_peak_mean == 0)
          *p_FSR_Ratio = 0;
        else
          *p_FSR_Ratio = fabs(p_steps_l->curr_voltage / p_steps_l->plant_peak_mean);



        if (*p_FSR_Ratio > (*p_Max_FSR_Ratio))
          (*p_Max_FSR_Ratio) = *p_FSR_Ratio; // update the max fsr Ratio


        // while updating the ratio value still continue to provide the control
        if ((p_steps_l->Setpoint ) > 0) { //depending on the leg the sign changes
          *p_Setpoint_Ankle_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint ) * (p_prop[0] * pow(*p_FSR_Ratio, 2) + p_prop[1] * (*p_FSR_Ratio) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2]));
          *p_Setpoint_Ankle_Pctrl_l = min(Max_Prop, *p_Setpoint_Ankle_Pctrl_l);
          if (abs(leg->Setpoint_Ankle_Pctrl) > abs(leg->MaxPropSetpoint)) {
            leg->MaxPropSetpoint = leg->Setpoint_Ankle_Pctrl; // Get max setpoint for current stance phase
          }
        }
        else if ((p_steps_l->Setpoint ) < 0) {
          *p_Setpoint_Ankle_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint ) * (p_prop[0] * pow(*p_FSR_Ratio, 2) + p_prop[1] * (*p_FSR_Ratio) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2]));
          *p_Setpoint_Ankle_Pctrl_l = min(Min_Prop, *p_Setpoint_Ankle_Pctrl_l);
          if (abs(leg->Setpoint_Ankle_Pctrl) > abs(leg->MaxPropSetpoint)) {
            leg->MaxPropSetpoint = leg->Setpoint_Ankle_Pctrl; // Get max setpoint for current stance phase
          }
        } else {
          *p_Setpoint_Ankle_Pctrl_l = 0;
          leg->MaxPropSetpoint = 0;
        }
      }
      if (Control_Mode_l == 4 || Control_Mode_l == 6) { // JOINT MOMENT CONTROL also known as pivot proportional control while taking the baseline
        // TN 7/15/19
        if (p_steps_l->plant_peak_mean == 0)
          *p_FSR_Ratio = 0;
        else
          *p_FSR_Ratio = fabs(p_steps_l->curr_voltage / p_steps_l->plant_peak_mean);



        if (*p_FSR_Ratio > (*p_Max_FSR_Ratio))
          (*p_Max_FSR_Ratio) = *p_FSR_Ratio; // update the max fsr Ratio


        // while updating the ratio value still continue to provide the control
        if ((p_steps_l->Setpoint ) > 0) { //depending on the leg the sign changes
          *p_Setpoint_Ankle_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint ) * (*p_FSR_Ratio));
          *p_Setpoint_Ankle_Pctrl_l = min(Max_Prop, *p_Setpoint_Ankle_Pctrl_l);
        }
        else if ((p_steps_l->Setpoint ) < 0) {
          *p_Setpoint_Ankle_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint ) * (*p_FSR_Ratio));
          *p_Setpoint_Ankle_Pctrl_l = min(Min_Prop, *p_Setpoint_Ankle_Pctrl_l);
        } else {
          *p_Setpoint_Ankle_Pctrl_l = 0;
        }
      }
    }

    return N3_l; //return the previous N3 value whis is not used
  }



  if (taking_baseline_l == 0 && p_steps_l->plant_peak_mean_temp != p_steps_l->plant_peak_mean) {
    p_steps_l->plant_peak_mean = p_steps_l->plant_peak_mean_temp;
    leg->baseline_value = p_steps_l->plant_peak_mean;
  }


  // if you transit from state 1 to state 3 dorsiflexion is completed and start plantarflexion
  if ((R_state_l == 3) && (R_state_old_l == 1 || R_state_old_l == 2))
  {

    // update the voltage peak to update torque in case of Bang Bang ctrl

    if (p_steps_l->curr_voltage > p_steps_l->peak)
      p_steps_l->peak =  p_steps_l->curr_voltage;

    // TN 7/15/19
    if (p_steps_l->plant_peak_mean == 0)
      *p_FSR_Ratio = 0;
    else
      *p_FSR_Ratio = fabs(p_steps_l->curr_voltage / p_steps_l->plant_peak_mean);

    if (*p_FSR_Ratio > (*p_Max_FSR_Ratio))
      (*p_Max_FSR_Ratio) = *p_FSR_Ratio;



    if (Control_Mode_l == 2) { // Balance control

      *p_Setpoint_Ankle_Pctrl_l = Balance_Torque_ref_based_on_Steady(leg);

      return N3_l; // No modification in the shaping which is disabled

    } else if (Control_Mode_l == 3) {
      // JOINT MOMENT CONTROL also known as pivot proportional control
      if ((p_steps_l->Setpoint ) > 0) {
        *p_Setpoint_Ankle_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint ) * (p_prop[0] * pow(*p_FSR_Ratio, 2) + p_prop[1] * (*p_FSR_Ratio) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2])); // the difference here is that we do it as a function of the FSR calibration
        *p_Setpoint_Ankle_Pctrl_l = min(Max_Prop, *p_Setpoint_Ankle_Pctrl_l);
        if (abs(leg->Setpoint_Ankle_Pctrl) > abs(leg->MaxPropSetpoint)) {
          leg->MaxPropSetpoint = leg->Setpoint_Ankle_Pctrl; // Get max setpoint for current stance phase
        }
      }
      else if ((p_steps_l->Setpoint ) < 0) {
        *p_Setpoint_Ankle_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint ) * (p_prop[0] * pow(*p_FSR_Ratio, 2) + p_prop[1] * (*p_FSR_Ratio) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2])); // the difference here is that we do it as a function of the FSR calibration
        *p_Setpoint_Ankle_Pctrl_l = min(Min_Prop, *p_Setpoint_Ankle_Pctrl_l);
        if (abs(leg->Setpoint_Ankle_Pctrl) > abs(leg->MaxPropSetpoint)) {
          leg->MaxPropSetpoint = leg->Setpoint_Ankle_Pctrl; // Get max setpoint for current stance phase
        }
      } else {
        *p_Setpoint_Ankle_Pctrl_l = 0;
        leg->MaxPropSetpoint = 0;
      }

      return N3_l; // No modification in the shaping function which is disabled
    }

    else if (Control_Mode_l == 4 || Control_Mode_l == 6)  {
      if ((p_steps_l->Setpoint ) > 0) {
        *p_Setpoint_Ankle_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint ) * (*p_FSR_Ratio)); // the difference here is that we do it as a function of the FSR calibration
        *p_Setpoint_Ankle_Pctrl_l = min(Max_Prop, *p_Setpoint_Ankle_Pctrl_l);
        if (abs(leg->Setpoint_Ankle_Pctrl) > abs(leg->MaxPropSetpoint)) {
          leg->MaxPropSetpoint = leg->Setpoint_Ankle_Pctrl; // Get max setpoint for current stance phase
        }
      }
      else if ((p_steps_l->Setpoint ) < 0) {
        *p_Setpoint_Ankle_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint ) * (*p_FSR_Ratio)); // the difference here is that we do it as a function of the FSR calibration
        *p_Setpoint_Ankle_Pctrl_l = min(Min_Prop, *p_Setpoint_Ankle_Pctrl_l);
        if (abs(leg->Setpoint_Ankle_Pctrl) > abs(leg->MaxPropSetpoint)) {
          leg->MaxPropSetpoint = leg->Setpoint_Ankle_Pctrl; // Get max setpoint for current stance phase
        }
      } else {
        *p_Setpoint_Ankle_Pctrl_l = 0;
      }
      return N3_l; // No modification in the shaping function which is disabled
    }

    // Otherwise we need to calculate the time

    // Parameters for speed adaption
    if (p_steps_l->flag_start_plant == false) // if it is true it means you started the step. Here I inizialize the parameters for speed adaption.
    {
      p_steps_l->plant_time = millis(); // start the plantarflexion
      p_steps_l->dorsi_time = millis() - (p_steps_l->dorsi_time); // calculate the dorsiflexion that has just finished

      if (p_steps_l->dorsi_time <= step_time_length / 4) // if <50ms probably it is noise
      {
        p_steps_l->peak = 0;
        p_steps_l->flag_start_plant = false;
        Serial.println(" SPD ADJ dorsi time too short ");
        return N3_l;
      }

      p_steps_l->flag_start_plant = true; // Parameters inizialized Start a step
    }


  }// end if you enter in state 3 from state 2 or 1

  // Hence here I am in state 2 or 1

  if (p_steps_l->flag_start_plant) { // If a step has started i.e. the states have passed from 1 or 2 to 3
    // if you transit from 3 to 1 plantar flexion is completed and start dorsiflexion

    if ((R_state_old_l == 3) && (R_state_l == 1 || R_state_l == 2)) {

      p_steps_l->count_plant++; // you have accomplished a step

      // start dorsiflexion
      p_steps_l->dorsi_time = millis();

      // calculate plantarflexion
      p_steps_l->plant_time = millis() - (p_steps_l->plant_time);

      if (p_steps_l->plant_time <= step_time_length)
      {

        p_steps_l->flag_start_plant = false;
        Serial.println(" SPD ADJ plant time too short ");
        return N3_l;
      }

      p_steps_l->flag_start_plant = false; // you have provided one step

      if (p_steps_l->count_plant >= 2) {
        // this is the time window of the filter for the plantarflexion
        if (p_steps_l->count_plant - 2 >= n_step_baseline) {
          for (int i = 0; i < n_step_baseline - 1; i++)
          {
            p_steps_l->four_step_plant_time[i] = p_steps_l->four_step_plant_time[i + 1];
          }
          p_steps_l->four_step_plant_time[n_step_baseline - 1] = p_steps_l->plant_time;
        }
        else {
          p_steps_l->four_step_plant_time[p_steps_l->count_plant - 2] = p_steps_l->plant_time;
        }

        // Update mean value
        p_steps_l->plant_mean = 0;
        for (int i = 0; i < n_step_baseline; i++) {
          p_steps_l->plant_mean += p_steps_l->four_step_plant_time[i];
        }

        p_steps_l->plant_mean = p_steps_l->plant_mean / n_step_baseline;
      }


    }//end if R old 3 i.e. finish plantarflexion
  }// end if flag_start_plant

  // During the all dorsiflexion set the voltage peak to 0, probably we just need to do it one time
  if (((R_state_l == 1) || (R_state_l == 2)) && R_state_old_l == 3) {
    p_steps_l->peak = 0;
    p_Max_FSR_Ratio = 0;
    *p_Setpoint_Ankle_Pctrl_l = New_PID_Setpoint_l; //Dorsiflexion setpoint GO 4/22/19
    if (leg->auto_KF_update == 0) {
      leg->MaxPropSetpoint = 0;
    }
  }
  return N3_l;
}
