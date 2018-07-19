////--------------------------------------------------------
//
//double Adj_FSR_th(int R_state_l, int R_state_old_l, steps* p_steps_l, double FSR_th_l, double New_PID_Setpoint_l, double ref_old_l) {
//
//  // every 4 steps compare them and adjust FSR_th
//  // it has be done starting from the first and only after FSR Calibration
//
//
//  if ((R_state_l == 3) && (R_state_old_l == 1 || R_state_old_l == 2) && (p_steps->flag_1_step == false))
//  {
//    //  Serial.println("first if");
//    //    double tcal_time = millis();
//    p_steps_l->flag_1_step = true;
//  }
//
//  if (p_steps_l->flag_1_step) {
//
//    // Identify changements in the states and hence a step
//
//    if ((R_state_old_l == 3) && (R_state_l == 1 || R_state_l == 2)) {
//      p_steps_l->flag_1_step = false;
//
//      (p_steps_l->n_steps)++;
//
//      Serial.print(" Old_PID_set = ");
//      Serial.println(Old_PID_Setpoint);
//      Serial.print(" sum torque_val ");
//      Serial.println(p_steps->torque_val);
//
//      p_steps_l->torque_val += PID_Setpoint;
//
//      if (((p_steps_l->n_steps) % 4) == 0) {
//        p_steps->torque_val_average = p_steps_l->torque_val / 4;
//        p_steps_l->torque_val = 0;
//        Serial.print(" Error ");
//        Serial.println(fabs(fabs(Setpoint_Ankle) - fabs(p_steps->torque_val_average)));
//
//
//        if (fabs(fabs(Setpoint_Ankle) - fabs(p_steps->torque_val_average)) >= 0.1)
//        {
//          Serial.print(" NOT ENOUGH ");
//          //
//          FSR_th_l = FSR_th_l - 0.05;
//          if (FSR_th_l < 0.5) FSR_th_l = 0.5;
//        } // end if Old PID Setpoint
//
//        Serial.print("FSR_th = ");
//        Serial.println(FSR_th_l);
//      }//end if 4 steps
//    }//end if R old 3
//  }// end if flag
//
//  return FSR_th_l;
//}
//
//
////--------------------------------------------------------
//
//double Adj_N3_speed(int R_state_l, int R_state_old_l, steps* p_steps_l, double N3_l, double New_PID_Setpoint_l, double N3_step) {
//
//  // every 4 steps compare them and adjust FSR_th
//  // it has be done starting from the first and only after FSR Calibration
//
//
//  if ((R_state_l == 3) && (R_state_old_l == 1 || R_state_old_l == 2) && (p_steps->flag_1_step == false))
//  {
//    //  Serial.println("first if");
//    //    double tcal_time = millis();
//    p_steps_l->flag_1_step = true;
//  }
//
//  if (p_steps_l->flag_1_step) {
//
//    // Identify changements in the states and hence a step
//
//    if ((R_state_old_l == 3) && (R_state_l == 1 || R_state_l == 2)) {
//      p_steps_l->flag_1_step = false;
//
//      (p_steps_l->n_steps)++;
//
//      Serial.print(" Old_PID_set = ");
//      Serial.println(Old_PID_Setpoint);
//      Serial.print(" sum torque_val ");
//      Serial.println(p_steps->torque_val);
//
//      p_steps_l->torque_val += PID_Setpoint;
//
//      if (((p_steps_l->n_steps) % 4) == 0) {
//        p_steps->torque_val_average = p_steps_l->torque_val / 4;
//        p_steps_l->torque_val = 0;
//        Serial.print(" Error ");
//        Serial.println(fabs(fabs(Setpoint_Ankle) - fabs(p_steps->torque_val_average)));
//
//
//        if (fabs(fabs(Setpoint_Ankle) - fabs(p_steps->torque_val_average)) >= 0.1)
//        {
//          Serial.print(" NOT ENOUGH ");
//          //
//          N3_l = N3_l - N3_step;
//          if (N3_l < 4) N3_l = 4;
//        } // end if Old PID Setpoint
//
//        Serial.print("N3 = ");
//        Serial.println(N3_l);
//      }//end if 4 steps
//    }//end if R old 3
//  }// end if flag
//
//  return N3_l;
//}
//
////---------------------------------------------------------------------------
//
//
//double Adj_N3_speed_with_time(int R_state_l, int R_state_old_l, steps* p_steps_l, double N3_l, double New_PID_Setpoint_l, double N3_step) {
//
//  // every 4 steps compare them and adjust FSR_th
//  // it has be done starting from the first and only after FSR Calibration
//
//
//  if ((R_state_l == 3) && (R_state_old_l == 1 || R_state_old_l == 2) && (p_steps_l->flag_1_step == false))
//  {
//
//
//    p_steps_l->plant_time = millis();
//    p_steps_l->dorsi_time = millis() - (p_steps_l->dorsi_time);
//    p_steps_l->flag_1_step = true;
//    Serial.print(" Dorsi time ");
//    Serial.println(p_steps->dorsi_time);
//
//
//  }
//
//  if (p_steps_l->flag_1_step) {
//
//    // Identify changements in the states and hence a step
//
//    if ((R_state_old_l == 3) && (R_state_l == 1 || R_state_l == 2)) {
//      p_steps_l->count_steps++;
//
//      if (p_steps_l->flag_N3_adjustment_time) {
//        if (p_steps_l->n_steps == 1) {
//          Serial.println(" N3 Adj activated");
//        }
//        p_steps_l->n_steps++;
//      }
//
//
//      p_steps_l->plant_time = millis() - (p_steps_l->plant_time);
//
//      if (p_steps_l->count_steps >= 2) {
//
//        if (p_steps_l->count_steps - 2 >= 4) {
//          for (int i = 0; i < 3; i++)
//          {
//            p_steps_l->four_step_dorsi_time[i] = p_steps_l->four_step_dorsi_time[i + 1];
//          }
//          p_steps_l->four_step_dorsi_time[3] = p_steps_l->dorsi_time;
//        }
//        else {
//          p_steps_l->four_step_dorsi_time[p_steps_l->count_steps - 2] = p_steps_l->dorsi_time;
//        }
//      }
//
//      p_steps_l->flag_1_step = false;
//      p_steps_l->dorsi_time = millis();
//      Serial.print(" Plant Time = ");
//      Serial.println(p_steps_l->plant_time);
//
//      if (p_steps_l->count_steps >= 2) {
//
//        if (p_steps_l->count_steps - 2 >= 4) {
//          for (int i = 0; i < 3; i++)
//          {
//            p_steps_l->four_step_plant_time[i] = p_steps_l->four_step_plant_time[i + 1];
//
//          }
//          p_steps_l->four_step_plant_time[3] = p_steps_l->plant_time;
//        }
//        else {
//          p_steps_l->four_step_plant_time[p_steps_l->count_steps - 2] = p_steps_l->plant_time;
//        }
//
//
//      }
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
//
//      // Now I would like to adjust N3 to be close 2/3 of the step duration. We don't need further FSR adjustement since
//      // faster means also with more force contact.
//      // The procedure should be this, if I send you a command like "take the mean"
//      // you take the average measurement and use that as reference, then you adjust N3 every 2/4 steps.
//
//      if (p_steps_l->flag_take_average) {
//        for (int i = 0; i < 4; i++) {
//          p_steps_l->dorsi_mean += p_steps_l->four_step_dorsi_time[i];
//        }
//        for (int i = 0; i < 4; i++) {
//          p_steps_l->plant_mean += p_steps_l->four_step_plant_time[i];
//        }
//        p_steps_l->dorsi_mean /= 4;
//        p_steps_l->plant_mean /= 4;
//        p_steps_l->flag_take_average = false;
//      }
//
//
//
//      if (p_steps_l->flag_N3_adjustment_time) {
//
//
//        if (((p_steps_l->n_steps) % 4) == 0) {
//          p_steps_l->dorsi_mean_old = p_steps_l->dorsi_mean;
//          p_steps_l->plant_mean_old = p_steps_l->plant_mean;
//
//          for (int i = 0; i < 4; i++) {
//            p_steps_l->dorsi_mean += p_steps_l->four_step_dorsi_time[i];
//          }
//          for (int i = 0; i < 4; i++) {
//            p_steps_l->plant_mean += p_steps_l->four_step_plant_time[i];
//          }
//          p_steps_l->dorsi_mean = p_steps_l->dorsi_mean / 4;
//          p_steps_l->plant_mean = p_steps_l->plant_mean / 4;
//
//          //          N3_l += (round(2 / 3 * (plant_mean)) - N3_l);
//          Serial.print(" mean = ");
//          Serial.println(p_steps_l->plant_mean);
//          N3_l = round((p_steps_l->plant_mean) * 1 / 2);
//          Serial.print(" N3 = ");
//          Serial.println(N3_l);
//          if (N3_l < 4) N3_l = 4;
//
//          p_steps_l->dorsi_mean = 0;
//          p_steps_l->plant_mean = 0;
//
//
//        }
//      }
//
//
//
//
//
//    }//end if R old 3
//  }// end if flag
//
//
//
//  return N3_l;
//}
//
//
////----------------------------------------------------
//
//
//double Adj_N3_speed_with_time_every_step(int R_state_l, int R_state_old_l, steps* p_steps_l, double N3_l, double New_PID_Setpoint_l, double N3_step) {
//
//  // every step compare them and adjust FSR_th
//  // it has be done starting from the first and only after FSR Calibration
//
//
//  // if you transit from state 1 to state 3 dorsiflexion is completed and start plantarflexion
//  if ((R_state_l == 3) && (R_state_old_l == 1 || R_state_old_l == 2) && (p_steps_l->flag_1_step == false))
//  {
//    p_steps_l->plant_time = millis(); // start the plantarflexion
//    p_steps_l->dorsi_time = millis() - (p_steps_l->dorsi_time); // calculate the dorsiflexion
//
//    if (p_steps_l->dorsi_time <= 30) p_steps_l->dorsi_time = 30;
//
//    p_steps_l->flag_1_step = true;
//    Serial.print(" Dorsi time ");
//    Serial.println(p_steps->dorsi_time);
//  }
//
//  if (p_steps_l->flag_1_step) {
//    // Identifying changes in the states I can identify a step
//    // if you transit from 3 to 1 plantar flexion is completed and start dorsiflexion
//    if ((R_state_old_l == 3) && (R_state_l == 1 || R_state_l == 2)) {
//
//      p_steps_l->count_steps++; // you have accomplished a step
//
//      // start dorsiflexion
//      p_steps_l->dorsi_time = millis();
//
//      if (p_steps_l->flag_N3_adjustment_time) {
//        // check for the first step in order to see if everything started properly
//        if (p_steps_l->n_steps == 1) {
//          Serial.println(" N3 Adj activated");
//        }
//        p_steps_l->n_steps++;
//      }
//
//      // calculate plantarflexion
//      p_steps_l->plant_time = millis() - (p_steps_l->plant_time);
//      if (p_steps_l->plant_time <= 16) p_steps_l->plant_time = 16;
//      p_steps_l->flag_1_step = false; // you have provided one step
//      Serial.print(" Plant Time = ");
//      Serial.println(p_steps_l->plant_time);
//
//      if (p_steps_l->count_steps >= 2) { //avoid the first step just to be sure
//
//        // this is the time window of the filter for the dorsiflexion
//        if (p_steps_l->count_steps - 2 >= 4) {
//          for (int i = 0; i < 3; i++)
//          {
//            p_steps_l->four_step_dorsi_time[i] = p_steps_l->four_step_dorsi_time[i + 1];
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
//          }
//          p_steps_l->four_step_plant_time[3] = p_steps_l->plant_time;
//        }
//        else {
//          p_steps_l->four_step_plant_time[p_steps_l->count_steps - 2] = p_steps_l->plant_time;
//        }
//
//        // Update mean value
//        p_steps_l->plant_mean = 0;
//        for (int i = 0; i < 4; i++) {
//          p_steps_l->plant_mean += p_steps_l->four_step_plant_time[i];
//        }
//        p_steps_l->plant_mean = p_steps_l->plant_mean / 4;
//      }
//
//
//      // print the last 4 results
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
//      if (p_steps_l->flag_N3_adjustment_time) {
//        N3_l = round((p_steps_l->plant_mean) * 0.3);
//        if (N3_l <= 4) N3_l = 4;
//        if (N3_l >= 500) N3_l = 500;
//
//        if (p_steps_l->torque_adj)
//        {
//
//          if (fabs(p_steps_l->plant_mean_base / p_steps_l->plant_mean) > 1.05)
//          {
//            Setpoint_Ankle = (p_steps_l->Setpoint) * (1 + ((fabs(p_steps_l->plant_mean_base / p_steps_l->plant_mean) - 1.05) / 7));
//
//            if (fabs(Setpoint_Ankle) - fabs(p_steps_l->Setpoint) > 3)
//            {
//              if (Setpoint_Ankle > 0)
//                Setpoint_Ankle = p_steps_l->Setpoint + 3;
//              if (Setpoint_Ankle < 0)
//                Setpoint_Ankle = p_steps_l->Setpoint - 3;
//            }
//          }
//          else
//          {
//            if (fabs(p_steps_l->plant_mean_base / p_steps_l->plant_mean) <= 0.90)
//            {
//              Setpoint_Ankle = (p_steps_l->Setpoint) * (fabs(p_steps_l->plant_mean_base / p_steps_l->plant_mean) + 0.08);
//
//              if (fabs(Setpoint_Ankle) < 1) {
//                if (Setpoint_Ankle > 0)
//                  Setpoint_Ankle = 1;
//                if (Setpoint_Ankle < 0)
//                  Setpoint_Ankle = -1;
//              }
//            }
//            else
//            {
//              Setpoint_Ankle = (p_steps_l->Setpoint);
//            }
//          }
//        }// end if torque_adj
//      }
//
//      Serial.print(" N3 = ");
//      Serial.print(N3_l);
//      Serial.print(" Trq = ");
//      Serial.println(Setpoint_Ankle);
//
//    }//end if R old 3 i.e. finish plantarflexion
//  }// end if flag_1_step
//
//
//  if (p_steps_l->flag_take_baseline) {
//    p_steps_l->dorsi_mean_base = 0;
//    p_steps_l->plant_mean_base = 0;
//    Setpoint_Ankle = p_steps_l->Setpoint;
//
//    for (int i = 0; i < 4; i++) {
//      p_steps_l->dorsi_mean_base += p_steps_l->four_step_dorsi_time[i];
//    }
//    for (int i = 0; i < 4; i++) {
//      p_steps_l->plant_mean_base += p_steps_l->four_step_plant_time[i];
//    }
//    p_steps_l->dorsi_mean_base /= 4;
//    p_steps_l->plant_mean_base /= 4;
//    p_steps_l->flag_take_baseline = false;
//    p_steps_l->torque_adj = true;
//    p_steps_l->Setpoint = Setpoint_Ankle;
//  }
//
//  return N3_l;
//}
//
////--------------------------------

double Adj_N3_speed_with_voltage_every_step(int R_state_l, int R_state_old_l, steps* p_steps_l, double N3_l, double New_PID_Setpoint_l, double* p_Setpoint_Ankle_l) {

  // every step compare them and adjust FSR_th
  // it has be done starting from the first and only after FSR Calibration


  // if you transit from state 1 to state 3 dorsiflexion is completed and start plantarflexion
  if ((R_state_l == 3) && (R_state_old_l == 1 || R_state_old_l == 2) && (p_steps_l->flag_1_step == false))
  {
    p_steps_l->plant_time = millis(); // start the plantarflexion
    p_steps_l->dorsi_time = millis() - (p_steps_l->dorsi_time); // calculate the dorsiflexion

    if (p_steps_l->dorsi_time <= 30) p_steps_l->dorsi_time = 30;

    p_steps_l->flag_1_step = true;
    Serial.print(" Dorsi time ");
    Serial.println(p_steps_l->dorsi_time);


    if (p_steps_l->torque_adj) //HERE YOU SHOULD TAKE THE PEAK VALUE DURING THE PLANTARFLEXION!!! MAYBE YOU CAN USE THE LAST STEP AS REFERENCE
    {

      //          Setpoint_Ankle = ((p_steps_l->curr_voltage) / (p_steps_l->voltage_ref * 0.9)) * p_steps_l->Setpoint;
      if (((p_steps_l->curr_voltage) - (p_steps_l->voltage_ref * 0.9)) > 0)
        *p_Setpoint_Ankle_l = fabs((p_steps_l->curr_voltage) - (p_steps_l->voltage_ref * 0.9)) * 1 + p_steps_l->Setpoint;

    }// end if torque_adj


  }

  if (p_steps_l->flag_1_step) {
    // Identifying changes in the states I can identify a step
    // if you transit from 3 to 1 plantar flexion is completed and start dorsiflexion
    if ((R_state_old_l == 3) && (R_state_l == 1 || R_state_l == 2)) {

      p_steps_l->count_steps++; // you have accomplished a step

      // start dorsiflexion
      p_steps_l->dorsi_time = millis();

      if (p_steps_l->flag_N3_adjustment_time) {
        // check for the first step in order to see if everything started properly
        if (p_steps_l->n_steps == 1) {
          Serial.println(" N3 Adj activated");
        }
        p_steps_l->n_steps++;
      }

      // calculate plantarflexion
      p_steps_l->plant_time = millis() - (p_steps_l->plant_time);
      if (p_steps_l->plant_time <= 16) p_steps_l->plant_time = 16;
      p_steps_l->flag_1_step = false; // you have provided one step
//      Serial.print(" Plant Time = ");
//      Serial.println(p_steps_l->plant_time);

      if (p_steps_l->count_steps >= 2) { //avoid the first step just to be sure

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
          }
          p_steps_l->four_step_plant_time[3] = p_steps_l->plant_time;
        }
        else {
          p_steps_l->four_step_plant_time[p_steps_l->count_steps - 2] = p_steps_l->plant_time;
        }

        // Update mean value
        p_steps_l->plant_mean = 0;
        for (int i = 0; i < 4; i++) {
          p_steps_l->plant_mean += p_steps_l->four_step_plant_time[i];
        }
        p_steps_l->plant_mean = p_steps_l->plant_mean / 4;
      }


      // print the last 4 results
      Serial.println();
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
        //
        //        if (p_steps_l->torque_adj)
        //        {
        //
        //          //          Setpoint_Ankle = ((p_steps_l->curr_voltage) / (p_steps_l->voltage_ref * 0.9)) * p_steps_l->Setpoint;
        //if (((p_steps_l->curr_voltage)-(p_steps_l->voltage_ref*0.9))>0)
        //          Setpoint_Ankle = fabs((p_steps_l->curr_voltage) - (p_steps_l->voltage_ref * 0.9))*1 + p_steps_l->Setpoint;
        //
        //        }// end if torque_adj
      }

//      Serial.print(" N3 = ");
//      Serial.println(N3_l);
//
//      Serial.print(" volt curr ref = ");
//
//      Serial.print((p_steps_l->curr_voltage));
//      Serial.print(",");
//      Serial.print((p_steps_l->voltage_ref * 0.9));
//
//
//      Serial.print(" Trq = ");
//      Serial.println(*p_Setpoint_Ankle_l);

    }//end if R old 3 i.e. finish plantarflexion
  }// end if flag_1_step


  if (p_steps_l->flag_take_baseline) {
    p_steps_l->dorsi_mean_base = 0;
    p_steps_l->plant_mean_base = 0;
    *p_Setpoint_Ankle_l = p_steps_l->Setpoint;

    for (int i = 0; i < 4; i++) {
      p_steps_l->dorsi_mean_base += p_steps_l->four_step_dorsi_time[i];
    }
    for (int i = 0; i < 4; i++) {
      p_steps_l->plant_mean_base += p_steps_l->four_step_plant_time[i];
    }
    p_steps_l->dorsi_mean_base /= 4;
    p_steps_l->plant_mean_base /= 4;
    p_steps_l->flag_take_baseline = false;
    p_steps_l->torque_adj = true;
    p_steps_l->Setpoint = *p_Setpoint_Ankle_l;
  }

  return N3_l;
}

