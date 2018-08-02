int take_baseline(int R_state_l, int R_state_old_l, steps* p_steps_l, int* p_flag_take_baseline_l, int address_baseline_l) {


  if ((R_state_l == 3) && (R_state_old_l == 1 || R_state_old_l == 2)) // I am in plantarflexion
  {

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
        Serial.println(" BASE Dorsi too short");
        return 0;
      } else {
        p_steps_l->flag_start_plant = true; // Parameters inizialized Start a step
        Serial.println(" BASE Start Plantar");
      }
    }
  }



  if (p_steps_l->flag_start_plant) {
    // If a step has started i.e. the states have passed from 1 or 2 to 3
    // if you transit from 3 to 1 plantar flexion is completed and start dorsiflexion

    if ((R_state_old_l == 3) && (R_state_l == 1 || R_state_l == 2)) {
      Serial.println(" BASE Start Dorsi");

      // start dorsiflexion

      p_steps_l->dorsi_time = millis();
      // calculate plantarflexion
      p_steps_l->plant_time = millis() - (p_steps_l->plant_time);

      if (p_steps_l->plant_time <= step_time_length)
      {
        p_steps_l->flag_start_plant = false;
        Serial.println("BASE Plant too short");
        return 0;
      } else {
        p_steps_l->flag_start_plant = false; // you have provided one plant
        Serial.println("Increase Plant number");
        p_steps_l->count_plant_base++; // you have accomplished a step
        Serial.println(p_steps_l->count_plant_base);
      }

      Serial.print(" BASE Plant Time = ");
      Serial.println(p_steps_l->plant_time);

      if ((p_steps_l->count_plant_base) >= 2) { // avoid the first step just to be sure

        //        // this is the time window of the filter for the dorsiflexion
        if (((p_steps_l->count_plant_base) - 2) >= n_step_baseline) {

          p_steps_l->dorsi_mean = 0;
          p_steps_l->plant_mean = 0;
          p_steps_l->plant_peak_mean = 0;


          for (int i = 0; i < n_step_baseline - 1; i++)
          {
            p_steps_l->four_step_dorsi_time[i] = p_steps_l->four_step_dorsi_time[i + 1];
            p_steps_l->dorsi_mean += p_steps_l->four_step_dorsi_time[i];

            p_steps_l->four_step_plant_time[i] = p_steps_l->four_step_plant_time[i + 1];
            p_steps_l->plant_mean += p_steps_l->four_step_plant_time[i];

            p_steps_l->four_step_plant_peak[i] = p_steps_l->four_step_plant_peak[i + 1];
            p_steps_l->plant_peak_mean += p_steps_l->four_step_plant_peak[i];
          }

          p_steps_l->four_step_dorsi_time[n_step_baseline - 1] = p_steps_l->dorsi_time;
          p_steps_l->dorsi_mean += p_steps_l->dorsi_time;

          p_steps_l->four_step_plant_time[n_step_baseline - 1] = p_steps_l->plant_time;
          p_steps_l->plant_mean += p_steps_l->plant_time;

          p_steps_l->four_step_plant_peak[n_step_baseline - 1] = p_steps_l->peak;
          p_steps_l->plant_peak_mean += p_steps_l->peak;

          p_steps_l->dorsi_mean = (p_steps_l->dorsi_mean) / n_step_baseline;
          p_steps_l->plant_mean = p_steps_l->plant_mean / n_step_baseline;
          p_steps_l->plant_peak_mean = 0.9 * (p_steps_l->plant_peak_mean / n_step_baseline);

          //          write_baseline(address_baseline_l, p_steps_l->plant_peak_mean);
          //          Serial.print("Baseline written in memory ");
          //          Serial.println(p_steps_l->plant_peak_mean);

          //HERE

          Serial.println("BASE before return");
          Serial.print(" Peak ");
          Serial.println(p_steps_l->peak);
          //          Serial.print(" Peak Mean");
          //          Serial.println(p_steps_l->plant_peak_mean);
          Serial.print(" N ");
          Serial.println(p_steps_l->count_plant_base);
        }
        else {
          p_steps_l->four_step_dorsi_time[p_steps_l->count_plant_base - 2] = p_steps_l->dorsi_time;
          //          p_steps_l->dorsi_mean += p_steps_l->dorsi_time;

          p_steps_l->four_step_plant_time[p_steps_l->count_plant_base - 2] = p_steps_l->plant_time;
          //          p_steps_l->plant_mean += p_steps_l->plant_time;

          p_steps_l->four_step_plant_peak[p_steps_l->count_plant_base - 2] = p_steps_l->peak;
          //          p_steps_l->plant_peak_mean += p_steps_l->peak;
          //          Serial.print("Step ");
          //          Serial.println((p_steps_l->count_plant_base - 2));
          //          Serial.print(" Peak ");
          //          Serial.println(p_steps_l->peak);
          Serial.println("Inside Peak vector ");

          for (int i = 0; i < n_step_baseline; i++) {
            Serial.println(p_steps_l->four_step_plant_peak[i]);
          }
        }

        //        p_steps_l->dorsi_mean = (p_steps_l->dorsi_mean) / 4;
        //        p_steps_l->plant_mean = p_steps_l->plant_mean / 4;
        //        p_steps_l->plant_peak_mean = p_steps_l->plant_peak_mean / 4;

        //        Serial.println("BASE before return");
        //        Serial.print(" Peak ");
        //        Serial.println(p_steps_l->peak);
        //        Serial.print(" Peak Mean");
        //        Serial.println(p_steps_l->plant_peak_mean);
        //        Serial.print(" N ");
        //        Serial.println(p_steps_l->count_plant_base);

        if (((p_steps_l->count_plant_base) - 2) >= n_step_baseline) {
          Serial.print("BASE return peak mean ");
          Serial.println(p_steps_l->plant_peak_mean);
        }
        //          p_steps_l->count_plant = 0;
        if (((p_steps_l->count_plant_base) - 2) >= n_step_baseline) {
          (p_steps_l->count_plant_base) = 0;
          *p_flag_take_baseline_l = 0;

          return (p_steps_l->count_plant_base);

        } // return 1 activate a flag that stops the calc of the baseline
      }// end if count_plant>2

    }//end dorsiflexion

  }// end start_step

  if (((R_state_l == 1) || (R_state_l == 2)) && R_state_old_l == 3)
    p_steps_l->peak = 0;
}// end take_baseline


//----------------------------------------------------------------------------------------------------
double Ctrl_ADJ(int R_state_l, int R_state_old_l, steps* p_steps_l, double N3_l, double New_PID_Setpoint_l, double* p_Setpoint_Ankle_l, double * p_Setpoint_Ankle_Pctrl_l, int flag_torque_time_volt_l, double prop_gain_l, double taking_baseline_l, double *p_FSR_Ratio_l, double *p_Max_FSR_Ratio_l) {

  // Speed adjustment -> the smoothing parameters are updated as a function of the plantar time in order to modify the shaping of the torque to the new step time.
  // It considers the average time of 4 steps. There's a filter of step_time_length on the plantarflexion time in order to cut the noise
  // The faster you go the more N3 decreases

  // Torque adjustment -> the torque set point is modified as a function of the voltage measured , the faster you go probably the bigger you hit the floor and hence
  // the voltage increases. It modifies the setpoint at each step.

  if (taking_baseline_l) {
    *p_Setpoint_Ankle_Pctrl_l = 0;
    *p_Setpoint_Ankle_l = 0;
    //    Serial.println("No ctrl adj");
    return N3_l;
  }

  // if you transit from state 1 to state 3 dorsiflexion is completed and start plantarflexion
  if ((R_state_l == 3) && (R_state_old_l == 1 || R_state_old_l == 2))
  {


    // update the voltage peak to update torque in case of Bang Bang ctrl
    if (p_steps_l->curr_voltage > p_steps_l->peak)
      p_steps_l->peak =  p_steps_l->curr_voltage;

    *p_FSR_Ratio_l = fabs(p_steps_l->curr_voltage / (p_steps_l->plant_peak_mean));
    if (*p_FSR_Ratio_l > (*p_Max_FSR_Ratio_l))
      (*p_Max_FSR_Ratio_l) = *p_FSR_Ratio_l;

    if (flag_torque_time_volt_l == 2) {
      if ((p_steps_l->Setpoint ) > 0) {
        *p_Setpoint_Ankle_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint ) * (prop_gain_l) * (*p_FSR_Ratio_l)); // the difference here is that we do it as a function of the FSR calibration
        *p_Setpoint_Ankle_Pctrl_l = min(Max_Prop, *p_Setpoint_Ankle_Pctrl_l);
      }
      else if ((p_steps_l->Setpoint ) < 0) {
        *p_Setpoint_Ankle_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint ) * (prop_gain_l) * (*p_FSR_Ratio_l)); // the difference here is that we do it as a function of the FSR calibration
        *p_Setpoint_Ankle_Pctrl_l = min(Min_Prop, *p_Setpoint_Ankle_Pctrl_l);
      } else {
        *p_Setpoint_Ankle_Pctrl_l = 0;
      }
      return N3_l; // No modification in the shaping

    } else if (flag_torque_time_volt_l == 3) {

      // Second version of pivot proportional Ctrl with polynomial law XXXXX QUI METTERE LEGGE
      if ((p_steps_l->Setpoint ) > 0) {
        *p_Setpoint_Ankle_Pctrl_l = max(Min_Prop, (p_steps_l->Setpoint ) * (p_prop[0] * pow(*p_FSR_Ratio_l, 2) + p_prop[1] * (*p_FSR_Ratio_l) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2])); // the difference here is that we do it as a function of the FSR calibration
        *p_Setpoint_Ankle_Pctrl_l = min(Max_Prop, *p_Setpoint_Ankle_Pctrl_l);
      }
      else if ((p_steps_l->Setpoint ) < 0) {
        *p_Setpoint_Ankle_Pctrl_l = max(-Max_Prop, (p_steps_l->Setpoint ) * (p_prop[0] * pow(*p_FSR_Ratio_l, 2) + p_prop[1] * (*p_FSR_Ratio_l) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2])); // the difference here is that we do it as a function of the FSR calibration
        *p_Setpoint_Ankle_Pctrl_l = min(Min_Prop, *p_Setpoint_Ankle_Pctrl_l);
      } else {
        *p_Setpoint_Ankle_Pctrl_l = 0;
      }

      return N3_l; // No modification in the shaping
    } else if (flag_torque_time_volt_l == 1) {


      Serial.print(" Ratios : ");
      Serial.print(*p_FSR_Ratio_l);
      Serial.print(" , ");
      Serial.print((*p_Max_FSR_Ratio_l));
      Serial.println();

      if ((p_steps_l->Setpoint ) > 0) {
        *p_Setpoint_Ankle_l = max(Min_Prop, (p_steps_l->Setpoint ) * (p_prop[0] * pow((*p_Max_FSR_Ratio_l), 2) + p_prop[1] * (*p_Max_FSR_Ratio_l) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2])); // the difference here is that we do it as a function of the FSR calibration
        *p_Setpoint_Ankle_l = min(Max_Prop, *p_Setpoint_Ankle_l);
      }
      else if ((p_steps_l->Setpoint ) < 0) {
        *p_Setpoint_Ankle_l = max(-Max_Prop, (p_steps_l->Setpoint ) * (p_prop[0] * pow((*p_Max_FSR_Ratio_l), 2) + p_prop[1] * (*p_Max_FSR_Ratio_l) + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2])); // the difference here is that we do it as a function of the FSR calibration
        *p_Setpoint_Ankle_l = min(Min_Prop, *p_Setpoint_Ankle_l);
      } else {
        *p_Setpoint_Ankle_l = 0;
      }     
      return 1;

      //      p_steps_l->flag_N3_adjustment_time = true;
      //      return N3_l;

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


    //    // Torque adaption as a function of the speed or of the pressure force
    //    // If you want to adjust the torque and hence torque_adj = 1
    //    //-----------------add 11:49 6/19/18
    //    if (p_steps_l->torque_adj)
    //    {
    //      if (p_steps_l->plant_time <= step_time_length)
    //      {
    //        p_steps_l->peak = 0;
    //        p_steps_l->flag_start_plant = false;
    //        Serial.println(" TRQ ADJ plant time too short ");
    //        return N3_l;
    //      }
    //
    //
    //      // if you use plantar time as reference to increase the torque
    //      if (flag_torque_time_volt_l == 0)
    //      {
    //        if (p_steps_l->flag_start_plant == true) {
    //          // if you're going use the time as reference to increase also the torque
    //
    //          if ((p_steps_l->Setpoint ) > 0) {
    //            *p_Setpoint_Ankle_l = max(Min_Prop, (p_steps_l->Setpoint ) * (1 / (fabs(p_steps_l->plant_mean / p_steps_l->plant_mean_base))));
    //            *p_Setpoint_Ankle_l = min(Max_Prop, *p_Setpoint_Ankle_l);
    //          }
    //          else if ((p_steps_l->Setpoint ) < 0) {
    //            *p_Setpoint_Ankle_l = max(-Max_Prop, (p_steps_l->Setpoint ) * (1 / (fabs(p_steps_l->plant_mean / p_steps_l->plant_mean_base))));
    //            *p_Setpoint_Ankle_l = min(Min_Prop, *p_Setpoint_Ankle_l);
    //          } else {
    //            *p_Setpoint_Ankle_l = 0;
    //          }
    //        }
    //      } else if (flag_torque_time_volt_l == 1) // If you use the volt or force value returned by the FSR sensors
    //      {
    //
    //        //        Serial.println("time_volt = 1");
    //        if (p_steps_l->flag_start_plant = true) {
    //          //          Serial.println("Start_plant = true");
    //          //  OLD VERSION before 07/24/18
    //          //          *p_Setpoint_Ankle_l = (p_steps_l->Setpoint ) * (fabs(p_steps_l->peak / p_steps_l->plant_peak_mean));
    //          //
    //          //          if ((p_steps_l->Setpoint ) > 0) {
    //          //            *p_Setpoint_Ankle_l = max(Min_Prop, (p_steps_l->Setpoint ) * (fabs(p_steps_l->peak / p_steps_l->plant_peak_mean)));
    //          //            *p_Setpoint_Ankle_l = min(Max_Prop, *p_Setpoint_Ankle_l);
    //          //          }
    //          //          else if ((p_steps_l->Setpoint ) < 0) {
    //          //            *p_Setpoint_Ankle_l = max(-Max_Prop, (p_steps_l->Setpoint ) * (fabs(p_steps_l->peak / p_steps_l->plant_peak_mean)));
    //          //            *p_Setpoint_Ankle_l = min(Min_Prop, *p_Setpoint_Ankle_l);
    //          //          } else {
    //          //            *p_Setpoint_Ankle_l = 0;
    //          //          }
    //          FSR_Ratio = fabs(p_steps_l->curr_voltage / p_steps_l->plant_peak_mean);
    //          if (FSR_Ratio > Max_FSR_Ratio)
    //            Max_FSR_Ratio = FSR_Ratio;
    //          Serial.println(Max_FSR_Ratio);
    //
    //          if ((p_steps_l->Setpoint ) > 0) {
    //            *p_Setpoint_Ankle_l = max(Min_Prop, (p_steps_l->Setpoint ) * (p_prop[0] * pow(Max_FSR_Ratio, 2) + p_prop[1] * Max_FSR_Ratio + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2])); // the difference here is that we do it as a function of the FSR calibration
    //            *p_Setpoint_Ankle_l = min(Max_Prop, *p_Setpoint_Ankle_l);
    //          }
    //          else if ((p_steps_l->Setpoint ) < 0) {
    //            *p_Setpoint_Ankle_l = max(-Max_Prop, (p_steps_l->Setpoint ) * (p_prop[0] * pow(Max_FSR_Ratio, 2) + p_prop[1] * Max_FSR_Ratio + p_prop[2]) / (p_prop[0] + p_prop[1] + p_prop[2])); // the difference here is that we do it as a function of the FSR calibration
    //            *p_Setpoint_Ankle_l = min(Min_Prop, *p_Setpoint_Ankle_l);
    //          } else {
    //            *p_Setpoint_Ankle_l = 0;
    //          }
    //
    //
    //
    //
    //        }// end if start_plant == true
    //        else {
    //          Max_FSR_Ratio = 0;
    //          Serial.println(Max_FSR_Ratio);
    //        }
    //      }// end if time_volt == 1
    //    }// end torque adj

  }// end if you enter in state 3 from state 2 or 1

  // Hence here I am in state 2 or 1

  if (p_steps_l->flag_start_plant) { // If a step has started i.e. the states have passed from 1 or 2 to 3
    // if you transit from 3 to 1 plantar flexion is completed and start dorsiflexion

    if ((R_state_old_l == 3) && (R_state_l == 1 || R_state_l == 2)) {


      *p_Max_FSR_Ratio_l = 0;
      p_steps_l->count_plant++; // you have accomplished a step

      // start dorsiflexion
      p_steps_l->dorsi_time = millis();

      if (p_steps_l->flag_N3_adjustment_time) { // If you wanted to adjust the smoothing as a function of the speed
        // check for the first step in order to see if everything started properly
        if (p_steps_l->n_steps == 1) {
          Serial.println(" N3 Adj activated");
        }
        p_steps_l->n_steps++;
      }

      // calculate plantarflexion
      p_steps_l->plant_time = millis() - (p_steps_l->plant_time);

      if (p_steps_l->plant_time <= step_time_length)
      {
        //        Serial.print(" Plant Time<200 probably noise ");
        //        Serial.println(p_steps_l->plant_time);

        p_steps_l->flag_start_plant = false;
        Serial.println(" SPD ADJ plant time too short ");
        return N3_l;
      }

      p_steps_l->flag_start_plant = false; // you have provided one step

      Serial.print(" Plant Time = ");
      Serial.println(p_steps_l->plant_time);

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

      if (p_steps_l->flag_N3_adjustment_time) {
        N3_l = round((p_steps_l->plant_mean) * p_steps_l->perc_l);

        if (N3_l <= 4) N3_l = 4;
        if (N3_l >= 500) N3_l = 500;
      }


      //            Serial.print(" Mean Volt ");
      //            Serial.println((p_steps_l->plant_mean_peak_base));
      //      Serial.print(" Actual peak ");
      //      Serial.println((p_steps_l->peak));
      //      Serial.print(" Actual Volt ");
      //      Serial.println((p_steps_l->curr_voltage));
      //      Serial.print(" Setpoint ");
      //      Serial.println(p_steps_l->Setpoint);
      Serial.print(" N3 = ");
      Serial.println(N3_l);
      //      Serial.print(" Trq = ");
      //      Serial.println(*p_Setpoint_Ankle_l);
      //      Serial.println(" ");


    }//end if R old 3 i.e. finish plantarflexion
  }// end if flag_start_plant

  // During the all dorsiflexion set the voltage peak to 0, probably we just need to do it one time
  if (((R_state_l == 1) || (R_state_l == 2)) && R_state_old_l == 3) {
    p_steps_l->peak = 0;
    p_Max_FSR_Ratio_l = 0;
  }

  return N3_l;
}
