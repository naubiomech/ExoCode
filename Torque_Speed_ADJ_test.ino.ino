//
//double Torque_ADJ(int R_state_l, int R_state_old_l, steps* p_steps_l, double N3_l, double New_PID_Setpoint_l, double* p_Setpoint_Ankle_l, int flag_torque_time_volt_l) {
//
//  // Speed adjustment -> the smoothing parameters are updated as a function of the plantar time in order to modify the shaping of the torque to the new step time.
//  // It considers the average time of 4 steps. There's a filter of step_time_length on the plantarflexion time in order to cut the noise
//  // The faster you go the more N3 decreases
//
//  // Torque adjustment -> the torque set point is modified as a function of the voltage measured , the faster you go probably the bigger you hit the floor and hence
//  // the voltage increases. It modifies the setpoint at each step.
//
//
//  // if you transit from state 1 to state 3 dorsiflexion is completed and start plantarflexion
//  if ((R_state_l == 3) && (R_state_old_l == 1 || R_state_old_l == 2))
//  {
//    // update the voltage peak
//    if (p_steps_l->curr_voltage > p_steps_l->peak)
//      p_steps_l->peak =  p_steps_l->curr_voltage;
//
//
//
//    // Parameters for speed adaption
//    if (p_steps_l->flag_1_step == false) // if it is true it means you started the step. Here I inizialize the parameters for speed adaption.
//    {
//      p_steps_l->plant_time = millis(); // start the plantarflexion
//      p_steps_l->dorsi_time = millis() - (p_steps_l->dorsi_time); // calculate the dorsiflexion that has just finished
//
//      if (p_steps_l->dorsi_time <= step_time_length / 4) // if <50ms probably it is noise
//      {
//        p_steps_l->peak = 0;
//        p_steps_l->flag_1_step = false;
//        Serial.println(" SPD ADJ dorsi time too short ");
//        return N3_l;
//      }
//
//      p_steps_l->flag_1_step = true; // Parameters inizialized Start a step
//    }
//
//
//    // Torque adaption as a function of the speed or of the pressure force
//    // If you want to adjust the torque and hence torque_adj = 1
//    if (p_steps_l->torque_adj)
//    {
//      if (p_steps_l->plant_time <= step_time_length)
//      {
//        p_steps_l->peak = 0;
//        p_steps_l->flag_1_step = false;
//        Serial.println(" TRQ ADJ plant time too short ");
//        return N3_l;
//      }
//
//      // if you use plantar time as reference to increase the torque
//      if (flag_torque_time_volt_l)
//      {
//        if (p_steps_l->flag_1_step = true) {
//          // if you're going use the time as reference to increase also the torque
//          *p_Setpoint_Ankle_l = (p_steps_l->Setpoint ) * (1 / (fabs(p_steps_l->plant_mean / p_steps_l->plant_mean_base)));
//        }
//      }
//      else // If you use the volt or force value returned by the FSR sensors
//      {
//        //                Serial.print(" Mean Volt ");
//        //                Serial.println((p_steps_l->plant_mean_peak_base));
//        //                Serial.print(" Actual peak ");
//        //                Serial.println((p_steps_l->peak));
//        if (p_steps_l->flag_1_step = true) {
//          *p_Setpoint_Ankle_l = (p_steps_l->Setpoint ) * (fabs(p_steps_l->peak / p_steps_l->plant_mean_peak_base));
////          *p_Setpoint_Ankle_l = (p_steps_l->Setpoint ) * (fabs(p_steps_l->plant_peak_mean / p_steps_l->plant_mean_peak_base));
//        }
//      } // end if torque adaption as a function of time or voltage
//
//      // Boundaries on Toque adaption
//      if (fabs(*p_Setpoint_Ankle_l) <= fabs(p_steps_l->Setpoint * 0.5)) {
//        Serial.println("Min Bound");
//        *p_Setpoint_Ankle_l = p_steps_l->Setpoint * 0.5;
//      }
//      if (fabs(*p_Setpoint_Ankle_l) >= fabs(p_steps_l->Setpoint * 1.5)) {
//        Serial.println("Max Bound");
//        *p_Setpoint_Ankle_l = p_steps_l->Setpoint * 1.5;
//      }
//
//    }// end if torque_adj
//  }// end if you enter in state 3 from state 2 or 1
//
//  // Hence here I am in state 2 or 1
//
//  if (p_steps_l->flag_1_step) { // If a step has started i.e. the states have passed from 1 or 2 to 3
//    // if you transit from 3 to 1 plantar flexion is completed and start dorsiflexion
//
//    if ((R_state_old_l == 3) && (R_state_l == 1 || R_state_l == 2)) {
//
//      p_steps_l->count_steps++; // you have accomplished a step
//
//      // start dorsiflexion
//      p_steps_l->dorsi_time = millis();
//
//      if (p_steps_l->flag_N3_adjustment_time) { // If you wanted to adjust the smoothing as a function of the speed
//        // check for the first step in order to see if everything started properly
//        if (p_steps_l->n_steps == 1) {
//          Serial.println(" N3 Adj activated");
//        }
//        p_steps_l->n_steps++;
//      }
//
//      // calculate plantarflexion
//      p_steps_l->plant_time = millis() - (p_steps_l->plant_time);
//
//      if (p_steps_l->plant_time <= step_time_length)
//      {
//        //        Serial.print(" Plant Time<200 probably noise ");
//        //        Serial.println(p_steps_l->plant_time);
//
//        p_steps_l->flag_1_step = false;
//        Serial.println(" SPD ADJ plant time too short ");
//        return N3_l;
//      }
//
//      p_steps_l->flag_1_step = false; // you have provided one step
//
//      Serial.print(" Plant Time = ");
//      Serial.println(p_steps_l->plant_time);
//
//      if (p_steps_l->count_steps >= 2) { // avoid the first step just to be sure
//
//        // this is the time window of the filter for the dorsiflexion
//        if (p_steps_l->count_steps - 2 >= 4) {
//          for (int i = 0; i < 3; i++)
//          {
//            p_steps_l->four_step_dorsi_time[i] = p_steps_l->four_step_dorsi_time[i + 1];
//
//          }
//          p_steps_l->four_step_dorsi_time[3] = p_steps_l->dorsi_time;
//        }
//        else {
//          p_steps_l->four_step_dorsi_time[p_steps_l->count_steps - 2] = p_steps_l->dorsi_time;
//        }
//
//        // Update mean value
//        p_steps_l->dorsi_mean = 0;
//        for (int i = 0; i < 4; i++) {
//          p_steps_l->dorsi_mean += p_steps_l->four_step_dorsi_time[i];
//        }
//        p_steps_l->dorsi_mean = (p_steps_l->dorsi_mean) / 4;
//      }
//
//      if (p_steps_l->count_steps >= 2) {
//        // this is the time window of the filter for the plantarflexion
//        if (p_steps_l->count_steps - 2 >= 4) {
//          for (int i = 0; i < 3; i++)
//          {
//            p_steps_l->four_step_plant_time[i] = p_steps_l->four_step_plant_time[i + 1];
//            p_steps_l->four_step_plant_peak[i] = p_steps_l->four_step_plant_peak[i + 1];
//          }
//          p_steps_l->four_step_plant_time[3] = p_steps_l->plant_time;
//          p_steps_l->four_step_plant_peak[3] = p_steps_l->peak;
//        }
//        else {
//          p_steps_l->four_step_plant_time[p_steps_l->count_steps - 2] = p_steps_l->plant_time;
//          p_steps_l->four_step_plant_peak[p_steps_l->count_steps - 2] = p_steps_l->peak;
//        }
//
//        // Update mean value
//        p_steps_l->plant_mean = 0;
//        p_steps_l->plant_peak_mean = 0;
//        for (int i = 0; i < 4; i++) {
//          p_steps_l->plant_mean += p_steps_l->four_step_plant_time[i];
//          p_steps_l->plant_peak_mean += p_steps_l->four_step_plant_peak[i];
//        }
//        p_steps_l->plant_mean = p_steps_l->plant_mean / 4;
//        p_steps_l->plant_peak_mean = p_steps_l->plant_peak_mean / 4;
//      }
//
//      // print the last 4 results
//      //      Serial.println();
//      //      for (int i = 0; i < 4; i++)
//      //      {
//      //        Serial.print("Dorsi ");
//      //        Serial.print(p_steps_l->four_step_dorsi_time[i]);
//      //        Serial.print(" ");
//      //      }
//      //      Serial.println(" ");
//      //      for (int i = 0; i < 4; i++)
//      //      {
//      //        Serial.print("Plant ");
//      //        Serial.print(p_steps_l->four_step_plant_time[i]);
//      //        Serial.print(" ");
//      //      }
//      //      Serial.println(" ");
//      //      Serial.print(" mean = ");
//      //      Serial.println(p_steps_l->plant_mean);
//
//      if (p_steps_l->flag_N3_adjustment_time) {
//        N3_l = round((p_steps_l->plant_mean) * p_steps_l->perc_l);
//        if (N3_l <= 4) N3_l = 4;
//        if (N3_l >= 500) N3_l = 500;
//      }
//
//      Serial.print(" Mean Volt ");
//      Serial.println((p_steps_l->plant_mean_peak_base));
//      Serial.print(" Actual peak ");
//      Serial.println((p_steps_l->peak));
//      Serial.print(" N3 = ");
//      Serial.println(N3_l);
//      Serial.print(" Trq = ");
//      Serial.println(*p_Setpoint_Ankle_l);
//
//
//
//      // For the torque adjustment take the baseline in time or in voltage
//      if (p_steps_l->flag_take_baseline) {
//
//        p_steps_l->plant_mean_peak_base = 0;
//        p_steps_l->dorsi_mean_base = 0;
//        p_steps_l->plant_mean_base = 0;
//
//        *p_Setpoint_Ankle_l = p_steps_l->Setpoint;
//
//        for (int i = 0; i < 4; i++) {
//          p_steps_l->dorsi_mean_base += p_steps_l->four_step_dorsi_time[i];
//        }
//        for (int i = 0; i < 4; i++) {
//          p_steps_l->plant_mean_base += p_steps_l->four_step_plant_time[i];
//          p_steps_l->plant_mean_peak_base += p_steps_l->four_step_plant_peak[i];
//        }
//        p_steps_l->dorsi_mean_base /= 4;
//        p_steps_l->plant_mean_base /= 4;
//        p_steps_l->plant_mean_peak_base /= 4;
//        p_steps_l->flag_take_baseline = false;
//        p_steps_l->torque_adj = true;
//        p_steps_l->Setpoint = *p_Setpoint_Ankle_l;
//      }
//
//
//    }//end if R old 3 i.e. finish plantarflexion
//  }// end if flag_1_step
//
//  // During the all dorsiflexion set the voltage peak to 0, probably we just need to do it one time
//  if (((R_state_l == 1) || (R_state_l == 2)) && R_state_old_l == 3)
//    p_steps_l->peak = 0;
//
//  //  // For the torque adjustment take the baseline in time or in voltage
//  //  if (p_steps_l->flag_take_baseline) {
//  //
//  //    p_steps_l->plant_mean_peak_base = 0;
//  //    p_steps_l->dorsi_mean_base = 0;
//  //    p_steps_l->plant_mean_base = 0;
//  //
//  //    *p_Setpoint_Ankle_l = p_steps_l->Setpoint;
//  //
//  //    for (int i = 0; i < 4; i++) {
//  //      p_steps_l->dorsi_mean_base += p_steps_l->four_step_dorsi_time[i];
//  //    }
//  //    for (int i = 0; i < 4; i++) {
//  //      p_steps_l->plant_mean_base += p_steps_l->four_step_plant_time[i];
//  //      p_steps_l->plant_mean_peak_base += p_steps_l->four_step_plant_peak[i];
//  //    }
//  //    p_steps_l->dorsi_mean_base /= 4;
//  //    p_steps_l->plant_mean_base /= 4;
//  //    p_steps_l->plant_mean_peak_base /= 4;
//  //    p_steps_l->flag_take_baseline = false;
//  //    p_steps_l->torque_adj = true;
//  //    p_steps_l->Setpoint = *p_Setpoint_Ankle_l;
//  //  }
//
//  return N3_l;
//}

//------------------------------------------------------


double Torque_ADJ(int R_state_l, int R_state_old_l, steps* p_steps_l, double N3_l, double New_PID_Setpoint_l, double* p_Setpoint_Ankle_l, double * p_Setpoint_Ankle_Pctrl_l, int flag_torque_time_volt_l) {

  // Speed adjustment -> the smoothing parameters are updated as a function of the plantar time in order to modify the shaping of the torque to the new step time.
  // It considers the average time of 4 steps. There's a filter of step_time_length on the plantarflexion time in order to cut the noise
  // The faster you go the more N3 decreases

  // Torque adjustment -> the torque set point is modified as a function of the voltage measured , the faster you go probably the bigger you hit the floor and hence
  // the voltage increases. It modifies the setpoint at each step.


  // if you transit from state 1 to state 3 dorsiflexion is completed and start plantarflexion
  if ((R_state_l == 3) && (R_state_old_l == 1 || R_state_old_l == 2))
  {


    // update the voltage peak
    if (p_steps_l->curr_voltage > p_steps_l->peak)
      p_steps_l->peak =  p_steps_l->curr_voltage;


    //    if (flag_torque_time_volt_l == 2 && p_steps_l->torque_adj == false) {
    //      p_steps_l->flag_take_baseline = true;
    //    }
    //    if (flag_torque_time_volt_l == 2 && p_steps_l->torque_adj == true) {
    //      *p_Setpoint_Ankle_Pctrl_l = (p_steps_l->Setpoint ) * (fabs( p_steps_l->curr_voltage / p_steps_l->plant_mean_peak_base));// the difference here is that if curr<peak during state 3 this will not decrease

    if (flag_torque_time_volt_l == 2) {
      *p_Setpoint_Ankle_Pctrl_l = (p_steps_l->Setpoint ) * (fabs( p_steps_l->curr_voltage / p_steps_l->voltage_ref));// the difference here is that we do it as a function of the FSR calibration
      Serial.print("Inside Trq_ADJ the voltage is:");
      Serial.println(p_steps_l->curr_voltage);
      //         New_PID_Setpoint_l=*p_Setpoint_Ankle_l;
      //      Serial.print(" Trq at beg a = ");
      //      Serial.println(*p_Setpoint_Ankle_Pctrl_l);
      //      p_steps_l->flag_1_step = true;
      //      Serial.print(" Mean Volt ");
      //      Serial.println((p_steps_l->plant_mean_peak_base));
      //      Serial.print(" Actual peak ");
      //      Serial.println((p_steps_l->peak));
      //      Serial.print(" Actual Volt ");
      //      Serial.println((p_steps_l->curr_voltage));
      //      Serial.print(" Setpoint ");
      //      Serial.println(p_steps_l->Setpoint);
      //      Serial.print(" N3 = ");
      //      Serial.println(N3_l);
      //      Serial.print(" Trq = ");
      //      Serial.println(*p_Setpoint_Ankle_l);
      //      Serial.println(" ");
      return N3_l;
    }
    // Parameters for speed adaption
    if (p_steps_l->flag_1_step == false) // if it is true it means you started the step. Here I inizialize the parameters for speed adaption.
    {
      p_steps_l->plant_time = millis(); // start the plantarflexion
      p_steps_l->dorsi_time = millis() - (p_steps_l->dorsi_time); // calculate the dorsiflexion that has just finished

      if (p_steps_l->dorsi_time <= step_time_length / 4) // if <50ms probably it is noise
      {
        p_steps_l->peak = 0;
        p_steps_l->flag_1_step = false;
        Serial.println(" SPD ADJ dorsi time too short ");
        return N3_l;
      }

      p_steps_l->flag_1_step = true; // Parameters inizialized Start a step
    }


    // Torque adaption as a function of the speed or of the pressure force
    // If you want to adjust the torque and hence torque_adj = 1
    if (p_steps_l->torque_adj)
    {
      if (p_steps_l->plant_time <= step_time_length)
      {
        p_steps_l->peak = 0;
        p_steps_l->flag_1_step = false;
        Serial.println(" TRQ ADJ plant time too short ");
        return N3_l;
      }

      // if you use plantar time as reference to increase the torque
      if (flag_torque_time_volt_l == 0)
      {
        if (p_steps_l->flag_1_step == true) {
          // if you're going use the time as reference to increase also the torque
          *p_Setpoint_Ankle_l = (p_steps_l->Setpoint ) * (1 / (fabs(p_steps_l->plant_mean / p_steps_l->plant_mean_base)));

          if (*p_Setpoint_Ankle_l >= Max_Prop)
            *p_Setpoint_Ankle_l = Max_Prop;
          if (*p_Setpoint_Ankle_l <= Min_Prop)
            *p_Setpoint_Ankle_l = Min_Prop;

        }
      } else if (flag_torque_time_volt_l == 1) // If you use the volt or force value returned by the FSR sensors
      {
        //                Serial.print(" Mean Volt ");
        //                Serial.println((p_steps_l->plant_mean_peak_base));
        //                Serial.print(" Actual peak ");
        //                Serial.println((p_steps_l->peak));
        if (p_steps_l->flag_1_step == true) {
          *p_Setpoint_Ankle_l = (p_steps_l->Setpoint ) * (fabs(p_steps_l->peak / p_steps_l->plant_mean_peak_base));
          //          *p_Setpoint_Ankle_l = (p_steps_l->Setpoint ) * (fabs(p_steps_l->plant_peak_mean / p_steps_l->plant_mean_peak_base));
        }
      } else if (flag_torque_time_volt_l == 2) {
        //        if (p_steps_l->flag_1_step == true) {
        //        *p_Setpoint_Ankle_l = (p_steps_l->Setpoint ) * (fabs( p_steps_l->curr_voltage / p_steps_l->plant_mean_peak_base));// the difference here is that if curr<peak during state 3 this will not decrease
        //        }

      } // end if torque adaption as a function of time or voltage

      // Boundaries on Toque adaption
      if (fabs(*p_Setpoint_Ankle_l) <= fabs(p_steps_l->Setpoint * 0.5)) {
        Serial.println("Min Bound");
        *p_Setpoint_Ankle_l = p_steps_l->Setpoint * 0.5;
      }
      if (fabs(*p_Setpoint_Ankle_l) >= fabs(p_steps_l->Setpoint * 1.5)) {
        Serial.println("Max Bound");
        *p_Setpoint_Ankle_l = p_steps_l->Setpoint * 1.5;
      }

    }// end if torque_adj
  }// end if you enter in state 3 from state 2 or 1

  // Hence here I am in state 2 or 1

  if (p_steps_l->flag_1_step) { // If a step has started i.e. the states have passed from 1 or 2 to 3
    // if you transit from 3 to 1 plantar flexion is completed and start dorsiflexion

    if ((R_state_old_l == 3) && (R_state_l == 1 || R_state_l == 2)) {

      p_steps_l->count_steps++; // you have accomplished a step

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

        p_steps_l->flag_1_step = false;
        Serial.println(" SPD ADJ plant time too short ");
        return N3_l;
      }

      p_steps_l->flag_1_step = false; // you have provided one step

      Serial.print(" Plant Time = ");
      Serial.println(p_steps_l->plant_time);

      if (p_steps_l->count_steps >= 2) { // avoid the first step just to be sure

        // this is the time window of the filter for the dorsiflexion
        if (p_steps_l->count_steps - 2 >= 4) {
          for (int i = 0; i < 3; i++)
          {
            p_steps_l->four_step_dorsi_time[i] = p_steps_l->four_step_dorsi_time[i + 1];

          }
          p_steps_l->four_step_dorsi_time[3] = p_steps_l->dorsi_time;
        }
        else {
          p_steps_l->four_step_dorsi_time[p_steps_l->count_steps - 2] = p_steps_l->dorsi_time;
        }

        // Update mean value
        p_steps_l->dorsi_mean = 0;
        for (int i = 0; i < 4; i++) {
          p_steps_l->dorsi_mean += p_steps_l->four_step_dorsi_time[i];
        }
        p_steps_l->dorsi_mean = (p_steps_l->dorsi_mean) / 4;
      }

      if (p_steps_l->count_steps >= 2) {
        // this is the time window of the filter for the plantarflexion
        if (p_steps_l->count_steps - 2 >= 4) {
          for (int i = 0; i < 3; i++)
          {
            p_steps_l->four_step_plant_time[i] = p_steps_l->four_step_plant_time[i + 1];
            p_steps_l->four_step_plant_peak[i] = p_steps_l->four_step_plant_peak[i + 1];
          }
          p_steps_l->four_step_plant_time[3] = p_steps_l->plant_time;
          p_steps_l->four_step_plant_peak[3] = p_steps_l->peak;
        }
        else {
          p_steps_l->four_step_plant_time[p_steps_l->count_steps - 2] = p_steps_l->plant_time;
          p_steps_l->four_step_plant_peak[p_steps_l->count_steps - 2] = p_steps_l->peak;
        }

        // Update mean value
        p_steps_l->plant_mean = 0;
        p_steps_l->plant_peak_mean = 0;
        for (int i = 0; i < 4; i++) {
          p_steps_l->plant_mean += p_steps_l->four_step_plant_time[i];
          p_steps_l->plant_peak_mean += p_steps_l->four_step_plant_peak[i];
        }
        p_steps_l->plant_mean = p_steps_l->plant_mean / 4;
        p_steps_l->plant_peak_mean = p_steps_l->plant_peak_mean / 4;
      }

      // print the last 4 results
      //      Serial.println();
      //      for (int i = 0; i < 4; i++)
      //      {
      //        Serial.print("Dorsi ");
      //        Serial.print(p_steps_l->four_step_dorsi_time[i]);
      //        Serial.print(" ");
      //      }
      //      Serial.println(" ");
      //      for (int i = 0; i < 4; i++)
      //      {
      //        Serial.print("Plant ");
      //        Serial.print(p_steps_l->four_step_plant_time[i]);
      //        Serial.print(" ");
      //      }
      //      Serial.println(" ");
      //      Serial.print(" mean = ");
      //      Serial.println(p_steps_l->plant_mean);

      if (p_steps_l->flag_N3_adjustment_time) {
        N3_l = round((p_steps_l->plant_mean) * p_steps_l->perc_l);
        if (N3_l <= 4) N3_l = 4;
        if (N3_l >= 500) N3_l = 500;
      }

      Serial.print(" Mean Volt ");
      Serial.println((p_steps_l->plant_mean_peak_base));
      Serial.print(" Actual peak ");
      Serial.println((p_steps_l->peak));
      Serial.print(" Actual Volt ");
      Serial.println((p_steps_l->curr_voltage));
      Serial.print(" Setpoint ");
      Serial.println(p_steps_l->Setpoint);
      Serial.print(" N3 = ");
      Serial.println(N3_l);
      Serial.print(" Trq = ");
      Serial.println(*p_Setpoint_Ankle_l);
      Serial.println(" ");



      // For the torque adjustment take the baseline in time or in voltage
      if (p_steps_l->flag_take_baseline) {

        p_steps_l->plant_mean_peak_base = 0;
        p_steps_l->dorsi_mean_base = 0;
        p_steps_l->plant_mean_base = 0;

        *p_Setpoint_Ankle_l = p_steps_l->Setpoint;

        for (int i = 0; i < 4; i++) {
          p_steps_l->dorsi_mean_base += p_steps_l->four_step_dorsi_time[i];
        }
        for (int i = 0; i < 4; i++) {
          p_steps_l->plant_mean_base += p_steps_l->four_step_plant_time[i];
          p_steps_l->plant_mean_peak_base += p_steps_l->four_step_plant_peak[i];
        }
        p_steps_l->dorsi_mean_base /= 4;
        p_steps_l->plant_mean_base /= 4;
        p_steps_l->plant_mean_peak_base /= 4;
        p_steps_l->flag_take_baseline = false;
        p_steps_l->torque_adj = true;
        p_steps_l->Setpoint = *p_Setpoint_Ankle_l;
      }


    }//end if R old 3 i.e. finish plantarflexion
  }// end if flag_1_step

  // During the all dorsiflexion set the voltage peak to 0, probably we just need to do it one time
  if (((R_state_l == 1) || (R_state_l == 2)) && R_state_old_l == 3)
    p_steps_l->peak = 0;

  //  // For the torque adjustment take the baseline in time or in voltage
  //  if (p_steps_l->flag_take_baseline) {
  //
  //    p_steps_l->plant_mean_peak_base = 0;
  //    p_steps_l->dorsi_mean_base = 0;
  //    p_steps_l->plant_mean_base = 0;
  //
  //    *p_Setpoint_Ankle_l = p_steps_l->Setpoint;
  //
  //    for (int i = 0; i < 4; i++) {
  //      p_steps_l->dorsi_mean_base += p_steps_l->four_step_dorsi_time[i];
  //    }
  //    for (int i = 0; i < 4; i++) {
  //      p_steps_l->plant_mean_base += p_steps_l->four_step_plant_time[i];
  //      p_steps_l->plant_mean_peak_base += p_steps_l->four_step_plant_peak[i];
  //    }
  //    p_steps_l->dorsi_mean_base /= 4;
  //    p_steps_l->plant_mean_base /= 4;
  //    p_steps_l->plant_mean_peak_base /= 4;
  //    p_steps_l->flag_take_baseline = false;
  //    p_steps_l->torque_adj = true;
  //    p_steps_l->Setpoint = *p_Setpoint_Ankle_l;
  //  }



  return N3_l;
}
