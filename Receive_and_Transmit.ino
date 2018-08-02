// Peek is the variable used to identify the message received by matlab
// To understand the commands see the file .......... in the folder


void receive_and_transmit()
{

  cmd_from_Gui = bluetooth.read();
  switch (cmd_from_Gui)
  {
    case '?':
      send_data_message_wc();
      break;

    case 'D':                                         //if MATLAB sent the character D
      *(data_to_send_point) = Setpoint_Ankle_LL;      //MATLAB is expecting to recieve the Torque Parameters
      send_command_message('D', data_to_send_point, 1);
      Serial.print("Received Left Set ");
      Serial.println(Setpoint_Ankle_LL);
      break;

    case 'd':
      *(data_to_send_point) = Setpoint_Ankle_RL;
      send_command_message('d', data_to_send_point, 1);     //MATLAB is expecting to receive the Torque Parameters
      Serial.print("Received Right Set");
      Serial.println(Setpoint_Ankle_RL);
      break;

    case 'F':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      receiveVals(8);                                                 //MATLAB is only sending 1 value, a double, which is 8 bytes
      Previous_Setpoint_Ankle_LL = Setpoint_Ankle_LL;
      memcpy(&Setpoint_Ankle_LL, &holdon, 8);                         //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the


      // New at 06/06/2018
      if (Setpoint_Ankle_LL < 0) {
        Setpoint_Ankle_LL = 0;
        Previous_Setpoint_Ankle_LL = 0;
        L_coef_in_3_steps = 0;
        L_activate_in_3_steps = 1;
        L_1st_step = 1;
        L_num_3_steps = 0;
        L_start_step = 0;
        Serial.println("Left Setpoint Negative, going to zero");
        // New at 06/04/2018
      } else {
        Setpoint_Ankle_LL = abs(Setpoint_Ankle_LL);                     //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
        //Recieved the large data chunk chopped into bytes, a roundabout way was needed
        L_p_steps->Setpoint = L_sign * Setpoint_Ankle_LL;
        Setpoint_Ankle_LL_Pctrl = Setpoint_Ankle_LL;
        L_activate_in_3_steps = 1;
        L_num_3_steps = 0;
        L_1st_step = 1;
        L_start_step = 0;
      }
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'f':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      receiveVals(8);                                                 //MATLAB is only sending 1 value, a double, which is 8 bytes
      Previous_Setpoint_Ankle_RL = Setpoint_Ankle_RL;
      memcpy(&Setpoint_Ankle_RL, &holdon, 8);                         //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      // New at 06/05/2018
      if (Setpoint_Ankle_RL < 0) {

        Setpoint_Ankle_RL = 0;
        Previous_Setpoint_Ankle_RL = 0;
        R_coef_in_3_steps = 0;
        R_activate_in_3_steps = 1;
        R_1st_step = 1;
        R_num_3_steps = 0;
        R_start_step = 0;
        Serial.println("Right Setpoint Negative, going to zero");

      } else {

        Setpoint_Ankle_RL = -abs(Setpoint_Ankle_RL);                    //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
        //Recieved the large data chunk chopped into bytes, a roundabout way was needed
        Setpoint_Ankle_RL_Pctrl = Setpoint_Ankle_RL;
        R_p_steps->Setpoint = R_sign * Setpoint_Ankle_RL;
        R_activate_in_3_steps = 1;
        R_num_3_steps = 0;
        R_1st_step = 1;
        R_start_step = 0;
      }
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'E':
      digitalWrite(onoff, HIGH);                                         //The GUI user is ready to start the trial so Motor is enabled
      stream = 1;                                                     //and the torque data is allowed to be streamed
      streamTimerCount = 0;
      timeElapsed = 0;
      break;

    case 'G':
      digitalWrite(onoff, LOW);                                         //The GUI user is ready to end the trial, so motor is disabled
      stream = 0;                                                    //and the torque data is no longer allowed to be streamed.
      break;

    case 'H':
      //digitalWrite(13, LOW);                                         //The user Wants to Recalibrate all the Sensors
      torque_calibration();
      write_torque_bias(address_torque_LL, Tcal_LL);
      write_torque_bias(address_torque_RL, Tcal_RL);
      break;

    case 'K':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      *(data_to_send_point) = kp_LL;
      *(data_to_send_point + 1) = kd_LL;
      *(data_to_send_point + 2) = ki_LL;
      send_command_message('K', data_to_send_point, 3);     //MATLAB is expecting to recieve the Torque Parameters
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'k':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      *(data_to_send_point) = kp_RL;
      *(data_to_send_point + 1) = kd_RL;
      *(data_to_send_point + 2) = ki_RL;
      send_command_message('k', data_to_send_point, 3);     //MATLAB is expecting to recieve the Torque Parameters
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'L':

      FSR_CAL_FLAG = 1;

      //
      //      // store_KF_LL = KF_LL;
      //      // store_KF_RL = KF_RL;
      //      // KF_RL = 0;
      //      KF_RL = 0;
      //
      //
      //      FSR_calibration();
      //      write_FSR_values(address_FSR_LL, fsr_Left_Heel_peak_ref);
      //      write_FSR_values((address_FSR_LL + sizeof(double) + sizeof(char)), fsr_Left_Toe_peak_ref);
      //      write_FSR_values(address_FSR_RL, fsr_Right_Heel_peak_ref);
      //      write_FSR_values((address_FSR_RL + sizeof(double) + sizeof(char)), fsr_Right_Toe_peak_ref);
      //
      //      L_p_steps->voltage_peak_ref = fsr_Left_Toe_peak_ref;
      //      R_p_steps->voltage_peak_ref = fsr_Right_Toe_peak_ref;
      //
      //      // KF_LL = store_KF_LL;
      //      // KF_RL = store_KF_RL;
      break;

    case 'M':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      receiveVals(24);                                                //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
      //MATLAB Sent Kp, then Kd, then Ki.
      memcpy(&kp_LL, holdOnPoint, 8);                                  //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      memcpy(&kd_LL, holdOnPoint + 8, 8);                              //memory space pointed to by the variable kp_L.  Essentially a roundabout way to change a variable value, but since the bluetooth
      memcpy(&ki_LL, holdOnPoint + 16, 8);                             //Recieved the large data chunk chopped into bytes, a roundabout way was needed
      PID_LL.SetTunings(kp_LL, ki_LL, kd_LL);
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'm':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      receiveVals(24);                                                //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
      //MATLAB Sent Kp, then Kd, then Ki.
      memcpy(&kp_RL, holdOnPoint, 8);                                  //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      memcpy(&kd_RL, holdOnPoint + 8, 8);                              //memory space pointed to by the variable kp_R.  Essentially a roundabout way to change a variable value, but since the bluetooth
      memcpy(&ki_RL, holdOnPoint + 16, 8);                             //Recieved the large data chunk chopped into bytes, a roundabout way was needed
      PID_RL.SetTunings(kp_RL, ki_RL, kd_RL);
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'N':
      //digitalWrite(13,LOW);
      *(data_to_send_point) = 0;
      *(data_to_send_point + 1) = 1;
      *(data_to_send_point + 2) = 2;
      send_command_message('N', data_to_send_point, 3);   //For the Arduino to prove to MATLAB that it is behaving, it will send back the character B
      break;

    case '<':
      if ((check_torque_bias(address_torque_LL)) && (check_torque_bias(address_torque_RL)))
      {
        Tcal_LL = read_torque_bias(address_torque_LL);
        Tcal_RL = read_torque_bias(address_torque_RL);
        Tarray_LL[3] = {0};
        Tarray_RL[3] = {0};
        *(data_to_send_point) = 1;
        //send_command_message('<', data_to_send_point, 0);   //For the Arduino to prove to MATLAB that it is behaving, it will send back the character B
      }
      else
      {
        *(data_to_send_point) = 0;
        //send_command_message('<', data_to_send_point, 0);   //For the Arduino to prove to MATLAB that it is behaving, it will send back the character B
      }
      if (((check_FSR_values(address_FSR_LL)) && (check_FSR_values(address_FSR_LL + sizeof(double) + 1))) &&
          ((check_FSR_values(address_FSR_RL)) && (check_FSR_values(address_FSR_RL + sizeof(double) + 1))))
      {
        //        fsr_Left_Heel_peak_ref = read_FSR_values(address_FSR_LL);
        //        fsr_Left_Toe_peak_ref = read_FSR_values(address_FSR_LL + sizeof(double) + 1);
        //        fsr_Right_Heel_peak_ref = read_FSR_values(address_FSR_RL);
        //        fsr_Right_Toe_peak_ref = read_FSR_values(address_FSR_RL + sizeof(double) + 1);

        fsr_Left_Combined_peak_ref = read_FSR_values(address_FSR_LL) + read_FSR_values(address_FSR_LL + sizeof(double) + 1);
        fsr_Right_Combined_peak_ref = read_FSR_values(address_FSR_RL) + read_FSR_values(address_FSR_RL + sizeof(double) + 1);

        *(data_to_send_point + 1) = 1;
        //send_command_message('<', data_to_send_point, 0);   //For the Arduino to prove to MATLAB that it is behaving, it will send back the character B
        Serial.print("Left values: ");
        Serial.print(fsr_Left_Combined_peak_ref);
        Serial.print(", ");
        Serial.print("Right values: ");
        Serial.print(fsr_Right_Combined_peak_ref);
      }
      else
      {
        *(data_to_send_point + 1) = 0;
        //send_command_message('*', data_to_send_point, 0);   //For the Arduino to prove to MATLAB that it is behaving, it will send back the character B
      }


      if (check_EXP_parameters(address_params))
      {
        read_all_params(address_params);
        *(data_to_send_point + 2) = 1;
      }
      else
      {
        *(data_to_send_point + 2) = 0;
      }

      send_command_message('<', data_to_send_point, 3);
      L_p_steps->voltage_peak_ref = fsr_Left_Combined_peak_ref;
      R_p_steps->voltage_peak_ref = fsr_Right_Combined_peak_ref;
      break;

    case '>':
      //------------------------------------------
      if (clean_torque_bias(address_torque_LL))
      {
        Serial.println("Clear Torque ");
      }
      else
      {
        Serial.println("No clear Torque");
      }
      if (clean_FSR_values(address_FSR_LL))
      {
        Serial.println("Clear FSR ");
      }
      else
      {
        Serial.println("No clear FSR");
      }
      //------------------------------------------
      if (clean_torque_bias(address_torque_RL))
      {
        Serial.println("Clear Torque ");
      }
      else
      {
        Serial.println("No clear Torque");
      }
      if (clean_FSR_values(address_FSR_RL))
      {
        Serial.println("Clear FSR ");
      }
      else
      {
        Serial.println("No clear FSR");
      }
      //------------------------------------------
      if (clean_FSR_values(address_FSR_LL + sizeof(double) + sizeof(char)))
      {
        Serial.println("Clear FSR ");
      }
      else
      {
        Serial.println("No clear FSR");
      }
      if (clean_FSR_values(address_FSR_RL + sizeof(double) + sizeof(char)))
      {
        Serial.println("Clear FSR ");
      }
      else
      {
        Serial.println("No clear FSR");
      }
      if (clean_EXP_Parameters(address_params))
      {
        Serial.println("Clear EXP params ");
      }
      else
      {
        Serial.println("No clear EXP params");
      }
      break;

    case '_':
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&KF_LL, &holdon, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      // KF_RL = store_KF_RL;
      break;

    case '-':
      // store_KF_LL = KF_LL;
      // KF_RL = 0;
      // KF_LL = 0;
      receiveVals(8);
      memcpy(&KF_RL, &holdon, 8);
      // KF_LL = store_KF_LL;
      break;

    case'`':
      *(data_to_send_point) = KF_LL;
      send_command_message('`', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
      Serial.print("Checking left KF ");
      Serial.println(KF_LL);
      break;

    case'~':
      *(data_to_send_point) = KF_RL;
      send_command_message('~', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
      Serial.print("Checking right KF ");
      Serial.println(KF_RL);
      break;

    case')':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      receiveVals(24);                               //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
      //MATLAB Sent N1 N2 and then N3 Paramenters for smoothing (see change pid setpoint)
      memcpy(&N1, holdOnPoint, 8);                   //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      memcpy(&N2, holdOnPoint + 8, 8);               //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
      memcpy(&N3, holdOnPoint + 16, 8);              //Recieved the large data chunk chopped into bytes, a roundabout way was needed

      N1_RL = N1;
      N2_RL = N2;
      N3_RL = N3;

      N1_LL = N1;
      N2_LL = N2;
      N3_LL = N3;

      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      Serial.print("Set Smooth ");
      Serial.print(" ");
      Serial.print(N1);
      Serial.print(" ");
      Serial.print(N2);
      Serial.print(" ");
      Serial.print(N3);
      Serial.println();
      break;

    case '(':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      *(data_to_send_point) = N1;
      *(data_to_send_point + 1) = N2;
      *(data_to_send_point + 2) = N3;
      send_command_message('(', data_to_send_point, 3);     //MATLAB is expecting to recieve the Torque Parameters
      Serial.print("Matlab Get Smooth ");
      Serial.print("");
      Serial.print(N1);
      Serial.print("");
      Serial.print(N2);
      Serial.print("");
      Serial.print(N3);
      Serial.println();

      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'P':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      //      L_p_steps->flag_take_baseline = true;
      L_p_steps->torque_adj = true;
      Old_Trq_time_volt = Trq_time_volt;
      Trq_time_volt = 1;
      Serial.println("Left Freq Baseline ");
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'p':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      //      R_p_steps->flag_take_baseline = true;
      R_p_steps->torque_adj = true;
      Old_Trq_time_volt = Trq_time_volt;
      Trq_time_volt = 1;
      Serial.println("Right Freq Baseline ");
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'O':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      L_p_steps->flag_N3_adjustment_time = true;
      Serial.println(" Left N3 Adj ");
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;


    case 'o':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      R_p_steps->flag_N3_adjustment_time = true;
      Serial.println(" Right N3 Adj ");
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'Q':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      *(data_to_send_point) = fsr_percent_thresh_Left_Toe;
      send_command_message('Q', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
      Serial.print("Checking the fsr_percent_thresh_Left_Toe: ");
      Serial.println(fsr_percent_thresh_Left_Toe);
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'q':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      *(data_to_send_point) = fsr_percent_thresh_Right_Toe;
      send_command_message('q', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
      Serial.print("Checking the fsr_percent_thresh_Right_Toe: ");
      Serial.println(fsr_percent_thresh_Right_Toe);
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'R':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&fsr_percent_thresh_Left_Toe, &holdon, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      L_p_steps->fsr_percent_thresh_Toe = fsr_percent_thresh_Left_Toe;
      Serial.print("Setting the fsr_percent_thresh_Left_Toe: ");
      Serial.println(fsr_percent_thresh_Left_Toe);
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'r':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&fsr_percent_thresh_Right_Toe, &holdon, 8);
      R_p_steps->fsr_percent_thresh_Toe = fsr_percent_thresh_Right_Toe;
      Serial.print("Setting the fsr_percent_thresh_Rigth_Toe: ");
      Serial.println(fsr_percent_thresh_Right_Toe);
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;//Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      break;

    case 'S':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&(L_p_steps->perc_l), &holdon, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      Serial.print("Setting the L_p_steps->perc_l: ");
      Serial.println(L_p_steps->perc_l);
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case's':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&(R_p_steps->perc_l), &holdon, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      Serial.print("Setting the R_p_steps->perc_l: ");
      Serial.println(R_p_steps->perc_l);
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'C':
      while (bluetooth.available() > 0) bluetooth.read();
      Serial.println("Buffer Clean");
      break;

    case 'T':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;

      L_p_steps->count_plant = 0;
      L_p_steps->n_steps = 0;
      L_p_steps->flag_start_plant = false;
      L_p_steps->flag_take_average = false;
      L_p_steps->flag_N3_adjustment_time = false;
      L_p_steps->flag_take_baseline = false;
      L_p_steps->torque_adj = false;
      Trq_time_volt = Old_Trq_time_volt;

      N3_LL = N3;
      Serial.print("Stop Left N3 adj, come back to: ");
      Serial.println(N3_LL);
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 't':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;


      R_p_steps->count_plant = 0;
      R_p_steps->n_steps = 0;
      R_p_steps->flag_start_plant = false;
      R_p_steps->flag_take_average = false;
      R_p_steps->flag_N3_adjustment_time = false;
      R_p_steps->flag_take_baseline = false;
      R_p_steps->torque_adj = false;
      Trq_time_volt = Old_Trq_time_volt;

      N3_RL = N3;
      Serial.print("Stop Right N3 adj, come back to: ");
      Serial.println(N3_RL);
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'I':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;

      *p_Setpoint_Ankle_LL = L_p_steps->Setpoint;
      L_p_steps->torque_adj = false;
      Serial.print("Stop Left TRQ adj, come back to: ");
      Serial.println(*p_Setpoint_Ankle_LL );
      Trq_time_volt = Old_Trq_time_volt;
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case 'i':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      //      cmd_from_Gui = bluetooth.read();

      *p_Setpoint_Ankle_RL = R_p_steps->Setpoint;
      R_p_steps->torque_adj = false;

      Serial.print("Stop Right TRQ adj, come back to: ");
      Serial.println(*p_Setpoint_Ankle_RL);
      Trq_time_volt = Old_Trq_time_volt;
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      break;

    case '!':
      if (stream == 1) {
        Serial.print("Cannot save data during streaming ");
      } else {
        Serial.print("Saving Experimental Parameters ");
        write_EXP_parameters(address_params);
      }//end if
      break;

    case 'W':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      L_sign = -1;
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      Serial.println(" Changed Sign in the Left torque ");
      break;

    case 'X':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      L_sign = 1;
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      Serial.println(" Restored the correct Sign the Left torque ");
      break;

    case 'w':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      R_sign = -1;
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      Serial.println(" Changed Sign in the Right torque ");
      break;

    case 'x':
      // store_KF_LL = KF_LL;
      // store_KF_RL = KF_RL;
      // KF_RL = 0;
      // KF_LL = 0;
      R_sign = 1;
      // KF_LL = store_KF_LL;
      // KF_RL = store_KF_RL;
      Serial.println(" Restored the correct Sign the Right torque ");
      break;

    case '[': // Receive Right Gain from GUI
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&R_Prop_Gain, &holdon, 8);
      Serial.print(" Settting Right Gain for Proportional Ctrl: ");
      Serial.println(R_Prop_Gain);
      break;

    case ']': // Send Right Gain to GUI
      *(data_to_send_point) = R_Prop_Gain;
      send_command_message(']', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
      Serial.print(" Checking Right Gain for Proportional Ctrl: ");
      Serial.println(R_Prop_Gain);
      break;

    case '{': // Receive Left Gain from GUI
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&L_Prop_Gain, &holdon, 8);
      Serial.print(" Settting Left Gain for Proportional Ctrl: ");
      Serial.println(L_Prop_Gain);
      break;

    case '}': // Send Left Gain to GUI
      *(data_to_send_point) = L_Prop_Gain;
      send_command_message('}', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
      Serial.print(" Checking Left Gain for Proportional Ctrl: ");
      Serial.println(L_Prop_Gain);
      break;

    case '+':
      Old_Trq_time_volt = Trq_time_volt;
      Trq_time_volt = 2;
      *p_Setpoint_Ankle_RL_Pctrl = R_p_steps->Setpoint;
      *p_Setpoint_Ankle_LL_Pctrl = L_p_steps->Setpoint;
      Serial.println(" Activate Proportional Ctrl: ");
      break;

    case '=':
      Trq_time_volt = Old_Trq_time_volt;
      R_p_steps->torque_adj = false;
      L_p_steps->torque_adj = false;
      *p_Setpoint_Ankle_RL = R_p_steps->Setpoint;
      *p_Setpoint_Ankle_LL = L_p_steps->Setpoint;
      *p_Setpoint_Ankle_RL_Pctrl = R_p_steps->Setpoint;
      *p_Setpoint_Ankle_LL_Pctrl = L_p_steps->Setpoint;
      Serial.println(" Deactivate Proportional Ctrl: ");
      break;


    case '.':
      flag_auto_KF = 1;
      KF_LL = 1;
      KF_RL = 1;
      Serial.println(" Activate Auto KF ");
      break;

    case ';':
      flag_auto_KF = 0;
      Serial.println(" Deactivate Auto KF ");
      break;

    case '#':
      Old_Trq_time_volt = Trq_time_volt;
      Trq_time_volt = 3; // activate pivot proportional control
      *p_Setpoint_Ankle_RL_Pctrl = R_p_steps->Setpoint;
      *p_Setpoint_Ankle_LL_Pctrl = L_p_steps->Setpoint;
      Serial.println(" Activate Proportional Pivot Ctrl ");
      break;

    case '^':
      Trq_time_volt = Old_Trq_time_volt;
      R_p_steps->torque_adj = false;
      L_p_steps->torque_adj = false;
      *p_Setpoint_Ankle_RL = R_p_steps->Setpoint;
      *p_Setpoint_Ankle_LL = L_p_steps->Setpoint;
      *p_Setpoint_Ankle_RL_Pctrl = R_p_steps->Setpoint;
      *p_Setpoint_Ankle_LL_Pctrl = L_p_steps->Setpoint;
      Serial.println(" Deactivate Proportional Pivot Ctrl ");
      break;

    case 'B':
      // check baseline
      Serial.println("Check Baseline");

//      L_p_steps->plant_peak_mean = read_baseline(L_baseline_address);
//      R_p_steps->plant_peak_mean = read_baseline(R_baseline_address);
      Serial.println(L_p_steps->plant_peak_mean);
      Serial.println(R_p_steps->plant_peak_mean);
      *(data_to_send_point) = L_p_steps->plant_peak_mean;
      *(data_to_send_point + 1) = R_p_steps->plant_peak_mean;
      send_command_message('B', data_to_send_point, 2);
      break;

    case 'b':
      // Calc baseline
      Serial.println(" Calc Baseline");
      FSR_baseline_FLAG_Left = 1;
      FSR_baseline_FLAG_Right = 1;
      base_1 = 0;
      base_2 = 0;
      L_p_steps->count_plant_base = 0;
      R_p_steps->count_plant_base = 0;
      R_p_steps->flag_start_plant = false;
      L_p_steps->flag_start_plant = false;
      R_p_steps->Setpoint = 0;
      L_p_steps->Setpoint = 0;
      break;

  }
  cmd_from_Gui = 0;
}





