// in this file there are the functions that modify the control strategy both in the strategy and in the shaping factors (N3) when required.


// take baseline for some of the controls during the plantarflexion state , i.e. 3
int take_baseline(Leg* leg, int R_state_l, int R_state_old_l, steps* p_steps_l, int* p_flag_take_baseline_l) {

  // update the voltage peak



  if (p_steps_l->curr_voltage > p_steps_l->peak)
    p_steps_l->peak =  p_steps_l->curr_voltage;


  if (p_steps_l->curr_voltage_Toe > p_steps_l->peak_Toe)
    p_steps_l->peak_Toe =  p_steps_l->curr_voltage_Toe;

  p_steps_l->Integ_Toe +=  p_steps_l->curr_voltage_Toe;   // SS  3/29/2020
  if (p_steps_l->Integ_Toe > p_steps_l->IntegMax_Toe)     // SS  3/29/2020
    p_steps_l->IntegMax_Toe =  p_steps_l->Integ_Toe;      // SS  3/29/2020

  if (p_steps_l->curr_voltage_Heel > p_steps_l->peak_Heel)
    p_steps_l->peak_Heel =  p_steps_l->curr_voltage_Heel;

  p_steps_l->Integ_Heel +=  p_steps_l->curr_voltage_Heel;   // SS  3/29/2020
  if (p_steps_l->Integ_Heel > p_steps_l->IntegMax_Heel)     // SS  3/29/2020
    p_steps_l->IntegMax_Heel =  p_steps_l->Integ_Heel;      // SS  3/29/2020

  if ((p_steps_l->curr_voltage_Heel - p_steps_l->curr_voltage_Toe) > p_steps_l->peak_HeelMinusToe)
    p_steps_l->peak_HeelMinusToe =  p_steps_l->curr_voltage_Heel - p_steps_l->curr_voltage_Toe;

  if ((p_steps_l->curr_voltage_Heel - p_steps_l->curr_voltage_Toe)>0)                                 // SS  3/29/2020
      p_steps_l->Integ_HeelMinusToe +=  (p_steps_l->curr_voltage_Heel - p_steps_l->curr_voltage_Toe); // SS  3/29/2020
  
  if (p_steps_l->Integ_HeelMinusToe > p_steps_l->IntegMax_HeelMinusToe)   // SS  3/29/2020
    p_steps_l->IntegMax_HeelMinusToe =  p_steps_l->Integ_HeelMinusToe;    // SS  3/29/2020

  if (p_steps_l->flag_start_plant == false) // if it is true it means you started the step. Here I inizialize the parameters for speed adaption.
  {
    p_steps_l->plant_time = millis(); // start the plantarflexion
    p_steps_l->dorsi_time = millis() - (p_steps_l->dorsi_time); // calculate the dorsiflexion that has just finished

    if (p_steps_l->dorsi_time <= step_time_length / 4) // if <50ms probably it is noise
    {
      p_steps_l->peak = 0;
      p_steps_l->peak_Toe = 0;   //  SS  9/18/2019
      p_steps_l->peak_Heel = 0;   //  SS  9/18/2019
      p_steps_l->peak_HeelMinusToe = 0;   // SS 9/10/2019
      p_steps_l->IntegMax_Toe = 0;           // SS  3/29/2020
      p_steps_l->IntegMax_Heel = 0;          // SS  3/29/2020
      p_steps_l->IntegMax_HeelMinusToe = 0;  // SS  3/29/2020
      p_steps_l->flag_start_plant = false;
      //        Serial.println(" BASE Dorsi too short");
      return 0;
    } else {
      p_steps_l->flag_start_plant = true; // Parameters inizialized Start a step
      //        Serial.println(" BASE Start Plantar");
      p_steps_l->Integ_Toe = 0;           // SS  3/29/2020 Reset the integral at the start of each step 
      p_steps_l->Integ_Heel = 0;          // SS  3/29/2020 Reset the integral at the start of each step 
      p_steps_l->Integ_HeelMinusToe = 0;  // SS  3/29/2020 Reset the integral at the start of each step 
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
          p_steps_l->plant_peak_mean_temp_Toe = 0;    //  SS  9/18/2019
          p_steps_l->plant_peak_mean_temp_Heel = 0;    //  SS  9/18/2019
          p_steps_l->plant_peak_mean_temp_HeelMinusToe = 0;  // SS 9/10/2019
          p_steps_l->plant_IntegMax_mean_temp_Toe = 0;           // SS 3/29/2020
          p_steps_l->plant_IntegMax_mean_temp_Heel = 0;           // SS 3/29/2020
          p_steps_l->plant_IntegMax_mean_temp_HeelMinusToe = 0;  // SS 3/29/2020


          for (int i = 0; i < n_step_baseline - 1; i++)
          {
            p_steps_l->four_step_dorsi_time[i] = p_steps_l->four_step_dorsi_time[i + 1];
            p_steps_l->dorsi_mean += p_steps_l->four_step_dorsi_time[i];

            p_steps_l->four_step_plant_time[i] = p_steps_l->four_step_plant_time[i + 1];
            p_steps_l->plant_mean += p_steps_l->four_step_plant_time[i];

            p_steps_l->four_step_plant_peak[i] = p_steps_l->four_step_plant_peak[i + 1];
            p_steps_l->plant_peak_mean_temp += p_steps_l->four_step_plant_peak[i];

            p_steps_l->four_step_plant_peak_Toe[i] = p_steps_l->four_step_plant_peak_Toe[i + 1];
            p_steps_l->plant_peak_mean_temp_Toe += p_steps_l->four_step_plant_peak_Toe[i];

            p_steps_l->four_step_plant_peak_Heel[i] = p_steps_l->four_step_plant_peak_Heel[i + 1];
            p_steps_l->plant_peak_mean_temp_Heel += p_steps_l->four_step_plant_peak_Heel[i];

            p_steps_l->four_step_plant_peak_HeelMinusToe[i] = p_steps_l->four_step_plant_peak_HeelMinusToe[i + 1];  // SS 9/10/2019
            p_steps_l->plant_peak_mean_temp_HeelMinusToe += p_steps_l->four_step_plant_peak_HeelMinusToe[i]; // SS 9/10/2019

            p_steps_l->four_step_plant_IntegMax_Toe[i] = p_steps_l->four_step_plant_IntegMax_Toe[i + 1];  //  SS  3/29/2020
            p_steps_l->plant_IntegMax_mean_temp_Toe += p_steps_l->four_step_plant_IntegMax_Toe[i];        //  SS  3/29/2020
            
            p_steps_l->four_step_plant_IntegMax_Heel[i] = p_steps_l->four_step_plant_IntegMax_Heel[i + 1];  //  SS  3/29/2020
            p_steps_l->plant_IntegMax_mean_temp_Heel += p_steps_l->four_step_plant_IntegMax_Heel[i];        //  SS  3/29/2020

            p_steps_l->four_step_plant_IntegMax_HeelMinusToe[i] = p_steps_l->four_step_plant_IntegMax_HeelMinusToe[i + 1];  //  SS  3/29/2020
            p_steps_l->plant_IntegMax_mean_temp_HeelMinusToe += p_steps_l->four_step_plant_IntegMax_HeelMinusToe[i];        //  SS  3/29/2020 
          }

          p_steps_l->four_step_dorsi_time[n_step_baseline - 1] = p_steps_l->dorsi_time;
          p_steps_l->dorsi_mean += p_steps_l->dorsi_time;

          p_steps_l->four_step_plant_time[n_step_baseline - 1] = p_steps_l->plant_time;
          p_steps_l->plant_mean += p_steps_l->plant_time;

          p_steps_l->four_step_plant_peak[n_step_baseline - 1] = p_steps_l->peak;
          p_steps_l->plant_peak_mean_temp += p_steps_l->peak;

          p_steps_l->four_step_plant_peak_Toe[n_step_baseline - 1] = p_steps_l->peak_Toe;
          p_steps_l->plant_peak_mean_temp_Toe += p_steps_l->peak_Toe;

          p_steps_l->four_step_plant_peak_Heel[n_step_baseline - 1] = p_steps_l->peak_Heel;
          p_steps_l->plant_peak_mean_temp_Heel += p_steps_l->peak_Heel;

          p_steps_l->four_step_plant_peak_HeelMinusToe[n_step_baseline - 1] = p_steps_l->peak_HeelMinusToe;  // SS 9/10/2019
          p_steps_l->plant_peak_mean_temp_HeelMinusToe += p_steps_l->peak_HeelMinusToe; // SS 9/10/2019

          p_steps_l->four_step_plant_IntegMax_Toe[n_step_baseline - 1] = p_steps_l->IntegMax_Toe; //  SS  3/29/2020
          p_steps_l->plant_IntegMax_mean_temp_Toe += p_steps_l->IntegMax_Toe;                     //  SS  3/29/2020

          p_steps_l->four_step_plant_IntegMax_Heel[n_step_baseline - 1] = p_steps_l->IntegMax_Heel; //  SS  3/29/2020
          p_steps_l->plant_IntegMax_mean_temp_Heel += p_steps_l->IntegMax_Heel;                     //  SS  3/29/2020

          p_steps_l->four_step_plant_IntegMax_HeelMinusToe[n_step_baseline - 1] = p_steps_l->IntegMax_HeelMinusToe; //  SS  3/29/2020
          p_steps_l->plant_IntegMax_mean_temp_HeelMinusToe += p_steps_l->IntegMax_HeelMinusToe;                     //  SS  3/29/2020

          p_steps_l->dorsi_mean = (p_steps_l->dorsi_mean) / n_step_baseline;
          p_steps_l->plant_mean = p_steps_l->plant_mean / n_step_baseline;
          p_steps_l->plant_peak_mean_temp = 1.0 * (p_steps_l->plant_peak_mean_temp) / n_step_baseline; //Changed from 0.9 to 1.0 by GO on 4/22/19


          p_steps_l->plant_peak_mean_temp_Toe =  (p_steps_l->plant_peak_mean_temp_Toe) / n_step_baseline ;  //  SS  9/18/2019

          p_steps_l->plant_peak_mean_temp_Heel = (p_steps_l->plant_peak_mean_temp_Heel) / n_step_baseline ;  //  SS  9/18/2019

          p_steps_l->plant_peak_mean_temp_HeelMinusToe = (p_steps_l->plant_peak_mean_temp_HeelMinusToe) / n_step_baseline ;  // SS 9/10/2019

          p_steps_l->plant_IntegMax_mean_temp_Toe = (p_steps_l->plant_IntegMax_mean_temp_Toe) / n_step_baseline ;  // SS  3/29/2020

          p_steps_l->plant_IntegMax_mean_temp_Heel = (p_steps_l->plant_IntegMax_mean_temp_Heel) / n_step_baseline ;  // SS  3/29/2020

          p_steps_l->plant_IntegMax_mean_temp_HeelMinusToe = (p_steps_l->plant_IntegMax_mean_temp_HeelMinusToe) / n_step_baseline ;  // SS  3/29/2020


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

          p_steps_l->four_step_plant_peak_Toe[p_steps_l->count_plant_base - 2] = p_steps_l->peak_Toe;  //  SS  9/18/2019

          p_steps_l->four_step_plant_peak_Heel[p_steps_l->count_plant_base - 2] = p_steps_l->peak_Heel;  //  SS  9/18/2019

          p_steps_l->four_step_plant_peak_HeelMinusToe[p_steps_l->count_plant_base - 2] = p_steps_l->peak_HeelMinusToe;  // SS 9/10/2019

          p_steps_l->four_step_plant_IntegMax_Toe[p_steps_l->count_plant_base - 2] = p_steps_l->IntegMax_Toe;  // SS 3/29/2020

          p_steps_l->four_step_plant_IntegMax_Heel[p_steps_l->count_plant_base - 2] = p_steps_l->IntegMax_Heel;  // SS 3/29/2020

          p_steps_l->four_step_plant_IntegMax_HeelMinusToe[p_steps_l->count_plant_base - 2] = p_steps_l->IntegMax_HeelMinusToe;  // SS 3/29/2020
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
          leg->baseline_value_Toe = p_steps_l->plant_peak_mean_Toe;
          leg->baseline_value_Heel = p_steps_l->plant_peak_mean_Heel;
          leg->baseline_value_HeelMinusToe = p_steps_l->plant_peak_mean_HeelMinusToe; // SS 9/10/2019
          leg->baseline_Integ_Toe = p_steps_l->plant_IntegMax_mean_Toe;                   //  SS  3/29/2020
          leg->baseline_Integ_Heel = p_steps_l->plant_IntegMax_mean_Heel;                 //  SS  3/29/2020
          leg->baseline_Integ_HeelMinusToe = p_steps_l->plant_IntegMax_mean_HeelMinusToe; //  SS  3/29/2020
          
          leg->baseline_value_Ankle = leg->baseline_value_Toe;  //  SS  9/18/2019
          leg->baseline_Integ_Ankle = leg->baseline_Integ_Toe;  //  SS  3/29/2020
          
          if (flag_id_KneeHeel){
            leg->baseline_value_Knee = leg->baseline_value_Heel;  //  SS  9/18/2019
            leg->baseline_Integ_Knee = leg->baseline_Integ_Heel;  //  SS  3/29/2020
          }else{
          leg->baseline_value_Knee = leg->baseline_value_HeelMinusToe;  //  SS  9/18/2019
          leg->baseline_Integ_Knee = leg->baseline_Integ_HeelMinusToe;  //  SS  3/29/2020
          }
          send_command_message('n', 0, 1); //GO 4/23/19 to communicate that baseline is done
          return (p_steps_l->count_plant_base);

        } // return 1 activate a flag that stops the calc of the baseline
      }// end if count_plant>2

    }//end dorsiflexion

  }// end start_step

  if (((R_state_l == 1) || (R_state_l == 2)) && R_state_old_l == 3) {
    p_steps_l->peak = 0;
    p_steps_l->peak_Toe = 0;  //  SS  9/18/2019
    p_steps_l->peak_Heel = 0;  //  SS  9/18/2019
    p_steps_l->peak_HeelMinusToe = 0; // SS 9/10/2019
    p_steps_l->IntegMax_Toe = 0; // SS 3/29/2020
    p_steps_l->IntegMax_Heel = 0; // SS 3/29/2020
    p_steps_l->IntegMax_HeelMinusToe = 0; // SS 3/29/2020
  }


}// end take_baseline



//------------------------------------------------------------------------------


double Control_Adjustment(Leg* leg, int R_state_l, int R_state_old_l, steps* p_steps_l, double N3_l, double New_PID_Setpoint_l, double * p_Setpoint_Ankle_Pctrl_l, double New_PID_Setpoint_Knee_l, double * p_Setpoint_Knee_Pctrl_l, int Control_Mode_l, double prop_gain_l, double Angle_Thigh_l, double Angle_Shank_l, double Angle_Foot_l, double taking_baseline_l, double *p_FSR_Ratio, double *p_FSR_Ratio_Toe, double *p_INTEG_Ratio_Toe, double *p_FSR_Ratio_Heel, double *p_INTEG_Ratio_Heel, double *p_FSR_Ratio_HeelMinusToe, double *p_INTEG_Ratio_HeelMinusToe, double *p_FSR_Ratio_Ankle, double *p_INTEG_Ratio_Ankle, double *p_Moment_Ratio_Ankle, double *p_FSR_Ratio_Knee, double *p_INTEG_Ratio_Knee, double* p_Max_FSR_Ratio, double* p_Max_FSR_Ratio_Toe, double* p_Max_INTEG_Ratio_Toe, double* p_Max_FSR_Ratio_Heel, double* p_Max_INTEG_Ratio_Heel, double* p_Max_FSR_Ratio_HeelMinusToe, double* p_Max_INTEG_Ratio_HeelMinusToe, double* p_Max_FSR_Ratio_Ankle, double* p_Max_INTEG_Ratio_Ankle, double* p_Max_FSR_Ratio_Knee, double* p_Max_INTEG_Ratio_Knee) {

  // Control Mode 2: Balance control
  // Control Mode 3: Joint Moment control, the torque is a percentage of the extimated Ankle moment. The mapping function that estimated the ankle moment use a ratio (p_FSR_Ratio) which depends on the current force of pressure
  // and the baseline value. The baseline value can be updated several times also during the execution of the task

  //otherwise Control Mode =  100 implies the classic bang bang whose shaping is based on N3
  //  Despite control mode 2 and 3 do not uses the N3 the function still returns a number associated to N3 which is not used.

  if (taking_baseline_l) { // if I am taking the baseline adapt some parameters for the controls

    //--------------------------------
    if (R_state_l == 3 && (R_state_old_l == 2 ||  R_state_old_l == 1))
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
        if ((p_steps_l->Setpoint_A ) > 0) { //depending on the leg the sign changes
          *p_Setpoint_Ankle_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint_A ) * (p_prop[0] * pow(*p_FSR_Ratio, 2) + p_prop[1] * (*p_FSR_Ratio) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2]));
          *p_Setpoint_Ankle_Pctrl_l = min(Max_Prop, *p_Setpoint_Ankle_Pctrl_l);
          if (abs(leg->Setpoint_Ankle_Pctrl) > abs(leg->MaxPropSetpoint)) {
            leg->MaxPropSetpoint = leg->Setpoint_Ankle_Pctrl; // Get max setpoint for current stance phase
          }
        }
        else if ((p_steps_l->Setpoint_A ) < 0) {
          *p_Setpoint_Ankle_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint_A ) * (p_prop[0] * pow(*p_FSR_Ratio, 2) + p_prop[1] * (*p_FSR_Ratio) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2]));
          *p_Setpoint_Ankle_Pctrl_l = min(Min_Prop, *p_Setpoint_Ankle_Pctrl_l);
          if (abs(leg->Setpoint_Ankle_Pctrl) > abs(leg->MaxPropSetpoint)) {
            leg->MaxPropSetpoint = leg->Setpoint_Ankle_Pctrl; // Get max setpoint for current stance phase
          }
        } else {
          *p_Setpoint_Ankle_Pctrl_l = 0;
          leg->MaxPropSetpoint = 0;
        }
      }
      if (Control_Mode_l == 4) { // JOINT MOMENT CONTROL also known as pivot proportional control while taking the baseline

          //  SS  9/18/2019
        //Ankle Control Setpoint
        if (p_steps_l->plant_peak_mean_Toe == 0)
          *p_FSR_Ratio_Toe = 0;
        else
          *p_FSR_Ratio_Toe = fabs(p_steps_l->curr_voltage_Toe / p_steps_l->plant_peak_mean_Toe);
        if (*p_FSR_Ratio_Toe > (*p_Max_FSR_Ratio_Toe))
          (*p_Max_FSR_Ratio_Toe) = *p_FSR_Ratio_Toe; // update the max fsr Ratio of Toe

        *p_FSR_Ratio_Ankle = *p_FSR_Ratio_Toe;

        if (p_steps_l->plant_IntegMax_mean_Toe == 0)  //  SS  3/29/2020
          *p_INTEG_Ratio_Toe = 0;                   //  SS  3/29/2020
        else
          *p_INTEG_Ratio_Toe = fabs(((p_steps_l->curr_voltage_Toe)*(leg->Angular_Impulse)) / p_steps_l->plant_IntegMax_mean_Toe);//  SS  3/29/2020
        
        if (*p_INTEG_Ratio_Toe > (*p_Max_INTEG_Ratio_Toe))                                          //  SS  3/29/2020
          (*p_Max_INTEG_Ratio_Toe) = *p_INTEG_Ratio_Toe;                                            //  SS  3/29/2020

        *p_INTEG_Ratio_Ankle = *p_INTEG_Ratio_Toe;    //  SS  3/29/2020
        *p_Moment_Ratio_Ankle =   ((p_steps_l->curr_voltage_Toe * A2T * FSR2Newton * ((sin(leg->Angle_Foot) * Mu) + cos(leg->Angle_Foot)))  //  SS  10/18/2020
                                - (p_steps_l->curr_voltage_Heel * A2H * FSR2Newton * ((cos(leg->Angle_Foot) * Mu) + sin(leg->Angle_Foot))))
                                / (p_steps_l->plant_peak_mean_Toe* A2T * FSR2Newton * ((0.3683 * Mu) + 0.9297));//sin(leg->Angle_Foot) and cos(leg->Angle_Foot) at the maximum AnkleMoment is 0.3683 and 0.9297. "curr_voltage_Heel" at the maximum AnkleMoment is 0
        
        // while updating the ratio value still continue to provide the control
        if ((p_steps_l->Setpoint_A ) > 0) { //depending on the leg the sign changes
//          *p_Setpoint_Ankle_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint_A ) * (*p_FSR_Ratio_Ankle));  //  SS  9/18/2019
//          *p_Setpoint_Ankle_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint_A ) * (*p_INTEG_Ratio_Ankle));  // SS  3/29/2020
          *p_Setpoint_Ankle_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint_A ) * (*p_Moment_Ratio_Ankle));  //  SS  10/18/2020
          *p_Setpoint_Ankle_Pctrl_l = min(Max_Prop, *p_Setpoint_Ankle_Pctrl_l);
          if (abs(leg->Setpoint_Ankle_Pctrl) > abs(leg->MaxPropSetpoint)) {
            leg->MaxPropSetpoint = leg->Setpoint_Ankle_Pctrl; // Get max setpoint for current stance phase
          }
        }
        else if ((p_steps_l->Setpoint_A ) < 0) {
//          *p_Setpoint_Ankle_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint_A ) * (*p_FSR_Ratio_Ankle));  //  SS  9/18/2019
//          *p_Setpoint_Ankle_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint_A ) * (*p_INTEG_Ratio_Ankle));   // SS  3/29/2020
          *p_Setpoint_Ankle_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint_A ) * (*p_Moment_Ratio_Ankle));  //  SS  10/18/2020
          *p_Setpoint_Ankle_Pctrl_l = min(Min_Prop, *p_Setpoint_Ankle_Pctrl_l);
          if (abs(leg->Setpoint_Ankle_Pctrl) > abs(leg->MaxPropSetpoint)) {
            leg->MaxPropSetpoint = leg->Setpoint_Ankle_Pctrl; // Get max setpoint for current stance phase
          }
        } else {
          *p_Setpoint_Ankle_Pctrl_l = 0;
          leg->MaxPropSetpoint = 0;
        }
        //Knee Control Setpoint
        // SS 9/10/2019
        if (p_steps_l->plant_peak_mean_Heel == 0)
          *p_FSR_Ratio_Heel = 0;
        else
          *p_FSR_Ratio_Heel = (p_steps_l->curr_voltage_Heel)  / p_steps_l->plant_peak_mean_Heel;  // SS 9/10/2019
        
        if (*p_FSR_Ratio_Heel < 0)  // SS  9/10/2019
            *p_FSR_Ratio_Heel = 0;  // SS 9/10/2019
            
        if (*p_FSR_Ratio_Heel > (*p_Max_FSR_Ratio_Heel))
          *p_Max_FSR_Ratio_Heel = *p_FSR_Ratio_Heel; // update the max fsr Ratio of Heel  // SS 9/10/2019

        if (p_steps_l->plant_IntegMax_mean_Heel == 0)  //  SS  3/29/2020
          *p_INTEG_Ratio_Heel = 0;                   //  SS  3/29/2020
        else
          *p_INTEG_Ratio_Heel = (((p_steps_l->curr_voltage_Heel)*(leg->Angular_Impulse_Knee)) / p_steps_l->plant_IntegMax_mean_Heel); // SS 6/8/2020

        if (*p_INTEG_Ratio_Heel < 0)  // SS  3/29/2020
            *p_INTEG_Ratio_Heel = 0;  // SS SS  3/29/2020
        
        if (*p_INTEG_Ratio_Heel > (*p_Max_INTEG_Ratio_Heel))                                          //  SS  3/29/2020
          (*p_Max_INTEG_Ratio_Heel) = *p_INTEG_Ratio_Heel;                                            //  SS  3/29/2020

                
        if (p_steps_l->plant_peak_mean_HeelMinusToe == 0)
          *p_FSR_Ratio_HeelMinusToe = 0;
        else
          *p_FSR_Ratio_HeelMinusToe = (p_steps_l->curr_voltage_Heel - p_steps_l->curr_voltage_Toe)  / p_steps_l->plant_peak_mean_HeelMinusToe;  // SS 9/10/2019
          if (*p_FSR_Ratio_HeelMinusToe < 0)  // SS  9/10/2019
            *p_FSR_Ratio_HeelMinusToe = 0;  // SS 9/10/2019
            
        if (*p_FSR_Ratio_HeelMinusToe > (*p_Max_FSR_Ratio_HeelMinusToe))
          *p_Max_FSR_Ratio_HeelMinusToe = *p_FSR_Ratio_HeelMinusToe; // update the max fsr ratio of HeelMinusToe  //  SS  3/29/2020

        if (p_steps_l->plant_IntegMax_mean_HeelMinusToe == 0)  //  SS  3/29/2020
          *p_INTEG_Ratio_HeelMinusToe = 0;                   //  SS  3/29/2020
        else
          *p_INTEG_Ratio_HeelMinusToe = (((p_steps_l->curr_voltage_Heel - p_steps_l->curr_voltage_Toe)*(leg->Angular_Impulse_Knee)) / p_steps_l->plant_IntegMax_mean_HeelMinusToe);//  SS  3/29/2020
          
        if (*p_INTEG_Ratio_HeelMinusToe <0)
            *p_INTEG_Ratio_HeelMinusToe = 0;
 
        if (*p_INTEG_Ratio_HeelMinusToe > (*p_Max_INTEG_Ratio_HeelMinusToe))                                          //  SS  3/29/2020
          (*p_Max_INTEG_Ratio_HeelMinusToe) = *p_INTEG_Ratio_HeelMinusToe;                                            //  SS  3/29/2020

        
        if (flag_id_KneeHeel){
          *p_FSR_Ratio_Knee = *p_FSR_Ratio_Heel;   //  SS  9/18/2019
          *p_Max_FSR_Ratio_Knee = *p_Max_FSR_Ratio_Heel;  //  SS  9/18/2019
          *p_INTEG_Ratio_Knee = *p_INTEG_Ratio_Heel;  //  SS  3/29/2020
          *p_Max_INTEG_Ratio_Knee = *p_Max_INTEG_Ratio_Heel;  //  SS  3/29/2020
        }else{
          *p_FSR_Ratio_Knee = *p_FSR_Ratio_HeelMinusToe;  //  SS  9/18/2019
          *p_Max_FSR_Ratio_Knee = *p_Max_FSR_Ratio_HeelMinusToe;  //  SS  9/18/2019
          *p_INTEG_Ratio_Knee = *p_INTEG_Ratio_HeelMinusToe;  //  SS  3/29/2020
          *p_Max_INTEG_Ratio_Knee = *p_Max_INTEG_Ratio_HeelMinusToe;  //  SS  3/29/2020
        }
        
        // while updating the ratio value still continue to provide the control 
        if ((p_steps_l->Setpoint_K ) > 0) { //depending on the leg the sign changes
//          *p_Setpoint_Knee_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint_K ) * (*p_FSR_Ratio_Knee));  // SS 9/10/2019
          *p_Setpoint_Knee_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint_K ) * (*p_INTEG_Ratio_Knee));  // SS 3/29/2020
          *p_Setpoint_Knee_Pctrl_l = min(Max_Prop, *p_Setpoint_Knee_Pctrl_l);
          if (abs(leg->Setpoint_Knee_Pctrl) > abs(leg->MaxPropSetpoint_Knee)) {  // SS 9/10/2019
            leg->MaxPropSetpoint_Knee = leg->Setpoint_Knee_Pctrl; // Get max setpoint for current stance phase
          }
        }
        else if ((p_steps_l->Setpoint_K ) < 0) {
//          *p_Setpoint_Knee_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint_K ) * (*p_FSR_Ratio_Knee));
          *p_Setpoint_Knee_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint_K ) * (*p_INTEG_Ratio_Knee));  // SS 3/29/2020
          *p_Setpoint_Knee_Pctrl_l = min(Min_Prop, *p_Setpoint_Knee_Pctrl_l);
          if (abs(leg->Setpoint_Knee_Pctrl) > abs(leg->MaxPropSetpoint_Knee)) {  // SS 9/10/2019
            leg->MaxPropSetpoint_Knee = leg->Setpoint_Knee_Pctrl; // Get max setpoint for current stance phase
          }
        } else {
          *p_Setpoint_Knee_Pctrl_l = 0;
          leg->MaxPropSetpoint_Knee = 0;  //  SS  9/18/2019
        }
        
      }

    }
// SS 9/10/2019
//    // TN 5/23/19
//    if (R_state_l == 2 && R_state_old_l == 1) {
//      if (Control_Mode_l == 4) {
//        //Knee Control Setpoint  // TN 5/8/19
//        // TN 7/15/19
//        if (p_steps_l->plant_peak_mean_Heel == 0)
//          *p_FSR_Ratio_Heel = 0;
//        else
//          *p_FSR_Ratio_Heel = fabs(p_steps_l->curr_voltage_Heel / p_steps_l->plant_peak_mean_Heel);
//        if (*p_FSR_Ratio_Heel > (*p_Max_FSR_Ratio_Heel))
//          (*p_Max_FSR_Ratio_Heel) = *p_FSR_Ratio_Heel; // update the max fsr Ratio of Heel
//
//
//        // while updating the ratio value still continue to provide the control  // TN 5/9/19
//        if ((p_steps_l->Setpoint_K ) > 0) { //depending on the leg the sign changes
//          *p_Setpoint_Knee_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint_K ) * (*p_FSR_Ratio_Heel));
//          *p_Setpoint_Knee_Pctrl_l = min(Max_Prop, *p_Setpoint_Knee_Pctrl_l);
//          if (abs(leg->Setpoint_Knee_Pctrl) > abs(leg->MaxPropSetpoint_Knee)) {  // TN 5/22/19
//            leg->MaxPropSetpoint_Knee = leg->Setpoint_Knee_Pctrl; // Get max setpoint for current stance phase
//          }
//        }
//        else if ((p_steps_l->Setpoint_K ) < 0) {
//          *p_Setpoint_Knee_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint_K ) * (*p_FSR_Ratio_Heel));
//          *p_Setpoint_Knee_Pctrl_l = min(Min_Prop, *p_Setpoint_Knee_Pctrl_l);
//          if (abs(leg->Setpoint_Knee_Pctrl) > abs(leg->MaxPropSetpoint_Knee)) {  // TN 5/22/19
//            leg->MaxPropSetpoint_Knee = leg->Setpoint_Knee_Pctrl; // Get max setpoint for current stance phase
//          }
//        } else {
//          *p_Setpoint_Knee_Pctrl_l = 0;
//          leg->MaxPropSetpoint_Knee = 0;  // TN 5/22/19
//        }
//
//      }
//
//    }

    return N3_l; //return the previous N3 value whis is not used
  }



  if (taking_baseline_l == 0 && p_steps_l->plant_peak_mean_temp != p_steps_l->plant_peak_mean) {
    p_steps_l->plant_peak_mean = p_steps_l->plant_peak_mean_temp;
    leg->baseline_value = p_steps_l->plant_peak_mean;
  }

  if (taking_baseline_l == 0 && p_steps_l->plant_peak_mean_temp_Toe != p_steps_l->plant_peak_mean_Toe) {
    p_steps_l->plant_peak_mean_Toe = p_steps_l->plant_peak_mean_temp_Toe;
    leg->baseline_value_Toe = p_steps_l->plant_peak_mean_Toe;
  }

  if (taking_baseline_l == 0 && p_steps_l->plant_peak_mean_temp_Heel != p_steps_l->plant_peak_mean_Heel) {
    p_steps_l->plant_peak_mean_Heel = p_steps_l->plant_peak_mean_temp_Heel;
    leg->baseline_value_Heel = p_steps_l->plant_peak_mean_Heel;
  }

  if (taking_baseline_l == 0 && p_steps_l->plant_peak_mean_temp_HeelMinusToe != p_steps_l->plant_peak_mean_HeelMinusToe) {  // SS 9/10/2019
    p_steps_l->plant_peak_mean_HeelMinusToe = p_steps_l->plant_peak_mean_temp_HeelMinusToe; // SS 9/10/2019
    leg->baseline_value_HeelMinusToe = p_steps_l->plant_peak_mean_HeelMinusToe; // SS 9/10/2019
  }

  if (taking_baseline_l == 0 && p_steps_l->plant_IntegMax_mean_temp_Toe != p_steps_l->plant_IntegMax_mean_Toe) {    //  SS  3/29/2020
    p_steps_l->plant_IntegMax_mean_Toe = p_steps_l->plant_IntegMax_mean_temp_Toe;
    leg->baseline_Integ_Toe = p_steps_l->plant_IntegMax_mean_Toe;
  }

  if (taking_baseline_l == 0 && p_steps_l->plant_IntegMax_mean_temp_Heel != p_steps_l->plant_IntegMax_mean_Heel) {    //  SS  3/29/2020
    p_steps_l->plant_IntegMax_mean_Heel = p_steps_l->plant_IntegMax_mean_temp_Heel;
    leg->baseline_Integ_Heel = p_steps_l->plant_IntegMax_mean_Heel;
  }

  if (taking_baseline_l == 0 && p_steps_l->plant_IntegMax_mean_temp_HeelMinusToe != p_steps_l->plant_IntegMax_mean_HeelMinusToe) {    //  SS  3/29/2020
    p_steps_l->plant_IntegMax_mean_HeelMinusToe = p_steps_l->plant_IntegMax_mean_temp_HeelMinusToe;
    leg->baseline_Integ_HeelMinusToe = p_steps_l->plant_IntegMax_mean_HeelMinusToe;
  }

  leg->baseline_value_Ankle = leg->baseline_value_Toe;
  leg->baseline_Integ_Ankle = leg->baseline_value_Toe;
  
  if (flag_id_KneeHeel){
    leg->baseline_value_Knee = leg->baseline_value_Heel;
    leg->baseline_Integ_Knee = leg->baseline_Integ_Heel;
  }else{
    leg->baseline_value_Knee = leg->baseline_value_HeelMinusToe;
    leg->baseline_Integ_Knee = leg->baseline_Integ_HeelMinusToe;
  }
  // if you transit from state 1 to state 3 dorsiflexion is completed and start plantarflexion
  if (R_state_l == 3 && (R_state_old_l == 2 || R_state_old_l == 1))
  {


    // update the voltage peak to update torque in case of Bang Bang ctrl

    if (p_steps_l->curr_voltage > p_steps_l->peak)
      p_steps_l->peak =  p_steps_l->curr_voltage;

    if (p_steps_l->plant_peak_mean == 0)
      *p_FSR_Ratio = 0;
    else
      *p_FSR_Ratio = fabs(p_steps_l->curr_voltage / p_steps_l->plant_peak_mean);

    if (*p_FSR_Ratio > (*p_Max_FSR_Ratio))
      (*p_Max_FSR_Ratio) = *p_FSR_Ratio;


    if (p_steps_l->curr_voltage_Toe > p_steps_l->peak_Toe)
      p_steps_l->peak_Toe =  p_steps_l->curr_voltage_Toe;
 
    if (p_steps_l->plant_peak_mean_Toe == 0){
      *p_FSR_Ratio_Toe = 0;
      *p_Moment_Ratio_Ankle = 0;  //  SS  10/18/2020
    }else{
      *p_FSR_Ratio_Toe = fabs(p_steps_l->curr_voltage_Toe / p_steps_l->plant_peak_mean_Toe);
      *p_Moment_Ratio_Ankle =   ((p_steps_l->curr_voltage_Toe * A2T * FSR2Newton * ((sin(leg->Angle_Foot) * Mu) + cos(leg->Angle_Foot)))  //  SS  10/18/2020
                                - (p_steps_l->curr_voltage_Heel * A2H * FSR2Newton * ((cos(leg->Angle_Foot) * Mu) + sin(leg->Angle_Foot))))
                                / (p_steps_l->plant_peak_mean_Toe* A2T * FSR2Newton * ((0.3683 * Mu) + 0.9297));//sin(leg->Angle_Foot) and cos(leg->Angle_Foot) at the maximum AnkleMoment is 0.3683 and 0.9297. "curr_voltage_Heel" at the maximum AnkleMoment is 0
    }
    
    if (*p_FSR_Ratio_Toe > (*p_Max_FSR_Ratio_Toe))
      (*p_Max_FSR_Ratio_Toe) = *p_FSR_Ratio_Toe;

    if (p_steps_l->plant_IntegMax_mean_Toe == 0)  // SS 3/29/2020
      *p_INTEG_Ratio_Toe = 0;
    else
      *p_INTEG_Ratio_Toe = fabs(((p_steps_l->curr_voltage_Toe) *(leg->Angular_Impulse))/ p_steps_l->plant_IntegMax_mean_Toe);// SS 3/29/2020

    if (*p_INTEG_Ratio_Toe > (*p_Max_INTEG_Ratio_Toe))
      (*p_Max_INTEG_Ratio_Toe) = *p_INTEG_Ratio_Toe;
      
      *p_FSR_Ratio_Ankle = *p_FSR_Ratio_Toe;// SS 2/11/2020
      *p_INTEG_Ratio_Ankle = *p_INTEG_Ratio_Toe;// SS 3/29/2020
      *p_Max_FSR_Ratio_Ankle = *p_Max_FSR_Ratio_Toe;
      *p_Max_INTEG_Ratio_Ankle = *p_Max_INTEG_Ratio_Toe;

    if (p_steps_l->curr_voltage_Heel > p_steps_l->peak_Heel)
      p_steps_l->peak_Heel =  p_steps_l->curr_voltage_Heel;

    if (p_steps_l->plant_peak_mean_Heel == 0)
      *p_FSR_Ratio_Heel = 0;
    else
      *p_FSR_Ratio_Heel = fabs(p_steps_l->curr_voltage_Heel / p_steps_l->plant_peak_mean_Heel);

    if (*p_FSR_Ratio_Heel > (*p_Max_FSR_Ratio_Heel))
      (*p_Max_FSR_Ratio_Heel) = *p_FSR_Ratio_Heel;

    if (p_steps_l->plant_IntegMax_mean_Heel == 0)  // SS 3/29/2020
      *p_INTEG_Ratio_Heel = 0;
    else
      *p_INTEG_Ratio_Heel = fabs(((p_steps_l->curr_voltage_Heel) *(leg->Angular_Impulse_Knee))/ p_steps_l->plant_IntegMax_mean_Heel); // SS 6/8/2020

    if (*p_INTEG_Ratio_Heel > (*p_Max_INTEG_Ratio_Heel))
      (*p_Max_INTEG_Ratio_Heel) = *p_INTEG_Ratio_Heel;  // SS 3/29/2020

    if ((p_steps_l->curr_voltage_Heel - p_steps_l->curr_voltage_Toe) > p_steps_l->peak_HeelMinusToe)  // SS 9/10/2019
      p_steps_l->peak_HeelMinusToe =  p_steps_l->curr_voltage_Heel - p_steps_l->curr_voltage_Toe ;  // SS 9/10/2019

    if (p_steps_l->plant_peak_mean_HeelMinusToe == 0)  // SS 9/10/2019
      *p_FSR_Ratio_HeelMinusToe = 0;  // SS 9/10/2019
    else
      *p_FSR_Ratio_HeelMinusToe = (p_steps_l->curr_voltage_Heel - p_steps_l->curr_voltage_Toe) / p_steps_l->plant_peak_mean_HeelMinusToe;  // SS 9/10/2019
    
    if(*p_FSR_Ratio_HeelMinusToe < 0) // SS 9/10/2019
        *p_FSR_Ratio_HeelMinusToe = 0;  // SS 9/10/2019
    
    if (*p_FSR_Ratio_HeelMinusToe > (*p_Max_FSR_Ratio_HeelMinusToe))  // SS 9/10/2019
      (*p_Max_FSR_Ratio_HeelMinusToe) = *p_FSR_Ratio_HeelMinusToe;  // SS 9/10/2019

    if (p_steps_l->plant_IntegMax_mean_HeelMinusToe == 0)  // SS 3/29/2020
      *p_INTEG_Ratio_HeelMinusToe = 0;
    else
      *p_INTEG_Ratio_HeelMinusToe = (((p_steps_l->curr_voltage_Heel - p_steps_l->curr_voltage_Toe) *(leg->Angular_Impulse_Knee))/ p_steps_l->plant_IntegMax_mean_HeelMinusToe); // SS 6/8/2020

    if (*p_INTEG_Ratio_HeelMinusToe < 0)
      *p_INTEG_Ratio_HeelMinusToe = 0;

    if (*p_INTEG_Ratio_HeelMinusToe > (*p_Max_INTEG_Ratio_HeelMinusToe))
      (*p_Max_INTEG_Ratio_HeelMinusToe) = *p_INTEG_Ratio_HeelMinusToe;  // SS 3/29/2020

    if (flag_id_KneeHeel){
      *p_FSR_Ratio_Knee = *p_FSR_Ratio_Heel;
      *p_Max_FSR_Ratio_Knee = *p_Max_FSR_Ratio_Heel;
      *p_INTEG_Ratio_Knee = *p_INTEG_Ratio_Heel;  // SS 3/29/2020
      *p_Max_INTEG_Ratio_Knee = *p_Max_INTEG_Ratio_Heel;  // SS 3/29/2020
    }else{
      *p_FSR_Ratio_Knee = *p_FSR_Ratio_HeelMinusToe;
      *p_Max_FSR_Ratio_Knee = *p_Max_FSR_Ratio_HeelMinusToe;
      *p_INTEG_Ratio_Knee = *p_INTEG_Ratio_HeelMinusToe;  // SS 3/29/2020
      *p_Max_INTEG_Ratio_Knee = *p_Max_INTEG_Ratio_HeelMinusToe;  // SS 3/29/2020
    }

    ////////

    if (Control_Mode_l == 2) { // Balance control

      *p_Setpoint_Ankle_Pctrl_l = Balance_Torque_ref_based_on_Steady(leg);

      return N3_l; // No modification in the shaping which is disabled

    } else if (Control_Mode_l == 3) {
      // JOINT MOMENT CONTROL also known as pivot proportional control
      if ((p_steps_l->Setpoint_A ) > 0) {
        *p_Setpoint_Ankle_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint_A ) * (p_prop[0] * pow(*p_FSR_Ratio, 2) + p_prop[1] * (*p_FSR_Ratio) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2])); // the difference here is that we do it as a function of the FSR calibration
        *p_Setpoint_Ankle_Pctrl_l = min(Max_Prop, *p_Setpoint_Ankle_Pctrl_l);
        if (abs(leg->Setpoint_Ankle_Pctrl) > abs(leg->MaxPropSetpoint)) {
          leg->MaxPropSetpoint = leg->Setpoint_Ankle_Pctrl; // Get max setpoint for current stance phase
        }
      }
      else if ((p_steps_l->Setpoint_A ) < 0) {
        *p_Setpoint_Ankle_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint_A ) * (p_prop[0] * pow(*p_FSR_Ratio, 2) + p_prop[1] * (*p_FSR_Ratio) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2])); // the difference here is that we do it as a function of the FSR calibration
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
    else if (Control_Mode_l == 4)  {

      if ((p_steps_l->Setpoint_A ) > 0) {
//        *p_Setpoint_Ankle_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint_A ) * (*p_FSR_Ratio_Ankle)); // the difference here is that we do it as a function of the FSR calibration  // TN 5/8/19
//        *p_Setpoint_Ankle_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint_A ) * (*p_INTEG_Ratio_Ankle)); //  SS  3/29/2020
        *p_Setpoint_Ankle_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint_A ) * (*p_Moment_Ratio_Ankle));  //  SS  10/18/2020
        *p_Setpoint_Ankle_Pctrl_l = min(Max_Prop, *p_Setpoint_Ankle_Pctrl_l);
        if (abs(leg->Setpoint_Ankle_Pctrl) > abs(leg->MaxPropSetpoint)) {
          leg->MaxPropSetpoint = leg->Setpoint_Ankle_Pctrl; // Get max setpoint for current stance phase
        }
      }
      else if ((p_steps_l->Setpoint_A ) < 0) {
//        *p_Setpoint_Ankle_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint_A ) * (*p_FSR_Ratio_Ankle));  // the difference here is that we do it as a function of the FSR calibration  // TN 5/8/19
//        *p_Setpoint_Ankle_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint_A ) * (*p_INTEG_Ratio_Ankle));  //  SS  3/29/2020
        *p_Setpoint_Ankle_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint_A ) * (*p_Moment_Ratio_Ankle));  //  SS  10/18/2020
        *p_Setpoint_Ankle_Pctrl_l = min(Min_Prop, *p_Setpoint_Ankle_Pctrl_l);
        if (abs(leg->Setpoint_Ankle_Pctrl) > abs(leg->MaxPropSetpoint)) {
          leg->MaxPropSetpoint = leg->Setpoint_Ankle_Pctrl; // Get max setpoint for current stance phase
        }
      } else {
        *p_Setpoint_Ankle_Pctrl_l = 0;
        leg->MaxPropSetpoint = 0;    //  SS  9/18/2019
      }

      // SS 9/10/2019
      if ((p_steps_l->Setpoint_K ) > 0) {
//        *p_Setpoint_Knee_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint_K ) * (*p_FSR_Ratio_Knee)); // the difference here is that we do it as a function of the FSR calibration  // SS 9/10/2019
        *p_Setpoint_Knee_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint_K ) * (*p_INTEG_Ratio_Knee));  //  SS  3/29/2020
        *p_Setpoint_Knee_Pctrl_l = min(Max_Prop, *p_Setpoint_Knee_Pctrl_l);
        if (abs(leg->Setpoint_Knee_Pctrl) > abs(leg->MaxPropSetpoint_Knee)) {
          leg->MaxPropSetpoint_Knee = leg->Setpoint_Knee_Pctrl; // Get max setpoint for current stance phase
        }
      }
      else if ((p_steps_l->Setpoint_K ) < 0) {
//        *p_Setpoint_Knee_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint_K ) * (*p_FSR_Ratio_Knee)); // the difference here is that we do it as a function of the FSR calibration  // SS 9/10/2019
        *p_Setpoint_Knee_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint_K ) * (*p_INTEG_Ratio_Knee));
        *p_Setpoint_Knee_Pctrl_l = min(Min_Prop, *p_Setpoint_Knee_Pctrl_l); // SS 9/17/2019
        if (abs(leg->Setpoint_Knee_Pctrl) > abs(leg->MaxPropSetpoint_Knee)) {
          leg->MaxPropSetpoint_Knee = leg->Setpoint_Knee_Pctrl; // Get max setpoint for current stance phase
        }
      } else {
        *p_Setpoint_Knee_Pctrl_l = 0;
        leg->MaxPropSetpoint_Knee = 0;  // SS 9/10/2019
      }

      return N3_l; // No modification in the shaping function which is disabled
    }


    // Otherwise we need to calculate the time

    // Parameters for speed adaption
    //    if (p_steps_l->flag_start_plant == false) // if it is true it means you started the step. Here I inizialize the parameters for speed adaption.
    //    {
    //      p_steps_l->plant_time = millis(); // start the plantarflexion
    //      p_steps_l->dorsi_time = millis() - (p_steps_l->dorsi_time); // calculate the dorsiflexion that has just finished
    //
    //      if (p_steps_l->dorsi_time <= step_time_length / 4) // if <50ms probably it is noise
    //      {
    //        p_steps_l->peak = 0;
    //        p_steps_l->peak_Toe = 0;  // TN 5/8/19
    //        p_steps_l->peak_Heel = 0;  // TN 5/8/19
    //        p_steps_l->flag_start_plant = false;
    //        //        Serial.println(" SPD ADJ dorsi time too short ");
    //        return N3_l;
    //      }
    //
    //      p_steps_l->flag_start_plant = true; // Parameters inizialized Start a step
    //    }




  }// end if you enter in state 3 from state 2 or 1


// SS 9/10/2019
//  // TN 5/23/19 transit from state 1 to state 2 for the knee control
//  if (R_state_l == 2 && R_state_old_l == 1) {
//
//    if (p_steps_l->curr_voltage_Heel > p_steps_l->peak_Heel)
//      p_steps_l->peak_Heel =  p_steps_l->curr_voltage_Heel;
//    // TN 7/15/19
//    if (p_steps_l->plant_peak_mean_Heel == 0)
//      *p_FSR_Ratio_Heel = 0;
//    else
//      *p_FSR_Ratio_Heel = fabs(p_steps_l->curr_voltage_Heel / p_steps_l->plant_peak_mean_Heel);
//
//    if (*p_FSR_Ratio_Heel > (*p_Max_FSR_Ratio_Heel))
//      (*p_Max_FSR_Ratio_Heel) = *p_FSR_Ratio_Heel;
//
//    if (Control_Mode_l == 4)  {
//
//      if ((p_steps_l->Setpoint_K ) > 0) {
//        *p_Setpoint_Knee_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint_K ) * (*p_FSR_Ratio_Heel)); // the difference here is that we do it as a function of the FSR calibration
//        *p_Setpoint_Knee_Pctrl_l = min(Max_Prop, *p_Setpoint_Knee_Pctrl_l);
//        if (abs(leg->Setpoint_Knee_Pctrl) > abs(leg->MaxPropSetpoint_Knee)) {
//          leg->MaxPropSetpoint_Knee = leg->Setpoint_Knee_Pctrl; // Get max setpoint for current stance phase
//        }
//      }
//      else if ((p_steps_l->Setpoint_K ) < 0) {
//
//        *p_Setpoint_Knee_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint_K ) * (*p_FSR_Ratio_Heel)); // the difference here is that we do it as a function of the FSR calibration
//        *p_Setpoint_Knee_Pctrl_l = min(Min_Prop, *p_Setpoint_Knee_Pctrl_l);
//        if (abs(leg->Setpoint_Knee_Pctrl) > abs(leg->MaxPropSetpoint_Knee)) {
//          leg->MaxPropSetpoint_Knee = leg->Setpoint_Knee_Pctrl; // Get max setpoint for current stance phase
//        }
//      } else {
//        *p_Setpoint_Knee_Pctrl_l = 0;
//        leg->MaxPropSetpoint_Knee = 0;
//      }
//      return N3_l; // No modification in the shaping function which is disabled
//    }
//
//    if (p_steps_l->flag_start_plant == false) // if it is true it means you started the step. Here I inizialize the parameters for speed adaption.
//    {
//      p_steps_l->plant_time = millis(); // start the plantarflexion
//      p_steps_l->dorsi_time = millis() - (p_steps_l->dorsi_time); // calculate the dorsiflexion that has just finished
//
//      if (p_steps_l->dorsi_time <= step_time_length / 4) // if <50ms probably it is noise
//      {
//        p_steps_l->peak = 0;
//        p_steps_l->peak_Toe = 0;  // TN 5/8/19
//        p_steps_l->peak_Heel = 0;  // TN 5/8/19
//        p_steps_l->flag_start_plant = false;
//        //        Serial.println(" SPD ADJ dorsi time too short ");
//        return N3_l;
//      }
//
//      p_steps_l->flag_start_plant = true; // Parameters inizialized Start a step
//    }
//
//
//  }


  // Hence here I am in state 2 or 1

  if (p_steps_l->flag_start_plant) { // If a step has started i.e. the states have passed from 1 or 2 to 3
    // if you transit from 3 to 1 plantar flexion is completed and start dorsiflexion

    if ((R_state_old_l == 3) && (R_state_l == 1) || (R_state_old_l == 3) && (R_state_l == 2)) {

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
  if (((R_state_l == 1) || (R_state_l == 2) ) && (R_state_old_l == 3)) {
    p_steps_l->peak = 0;
    p_steps_l->peak_Toe = 0;  //  SS  9/18/2019
    p_steps_l->peak_Heel = 0;  //  SS  9/18/2019
    p_steps_l->peak_HeelMinusToe = 0;   // SS 9/10/2019
    p_steps_l->IntegMax_Toe = 0;           // SS 3/29/2020
    p_steps_l->IntegMax_Heel = 0;           // SS 3/29/2020
    p_steps_l->IntegMax_HeelMinusToe = 0;   // SS 3/29/2020

    p_Max_FSR_Ratio = 0;
    p_Max_FSR_Ratio_Toe = 0;   //  SS  9/18/2019
    p_Max_FSR_Ratio_Heel = 0;  //  SS  9/18/2019
    p_Max_FSR_Ratio_HeelMinusToe = 0;   // SS 9/10/2019
    p_Max_INTEG_Ratio_Toe = 0;           // SS 3/29/2020
    p_Max_INTEG_Ratio_Heel = 0;           // SS 3/29/2020
    p_Max_INTEG_Ratio_HeelMinusToe = 0;   // SS 3/29/2020
    p_Max_FSR_Ratio_Ankle = 0;   // SS 2/5/2020
    p_Max_FSR_Ratio_Knee = 0;   // SS 2/5/2020
    p_Max_INTEG_Ratio_Ankle = 0;   // SS 3/29/2020
    p_Max_INTEG_Ratio_Knee = 0;   // SS 3/29/2020
    *p_Setpoint_Ankle_Pctrl_l = New_PID_Setpoint_l; //Dorsiflexion setpoint GO 4/22/19
    *p_Setpoint_Knee_Pctrl_l = New_PID_Setpoint_Knee_l; //Dorsiflexion setpoint  //  SS  9/18/2019
    if (leg->auto_KF_update == 0) {
      leg->MaxPropSetpoint = 0;
      leg->MaxPropSetpoint_Knee = 0;  //  SS  9/18/2019
    }
  }
  return N3_l;
}
