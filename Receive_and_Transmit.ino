// cmd_from_Gui is the variable used to identify the message received by matlab

double app = 0;

void receive_and_transmit()
{

  cmd_from_Gui = bluetooth.read();
  switch (cmd_from_Gui)
  {
    case '?':
      send_data_message_wc();
      break;


    case 'Y':  //ZL ADDED THIS TO RECIEVE MATLAB COMMAND TO FLUSH BLUETOOTH FROM GUI SWITCH
      bluetooth.flush();
      break;


    case 'D':        // TN 7/3/19
      if (Flag_Ankle_Cfg == true) {
        *(data_to_send_point) = left_leg->Setpoint_Ankle;      //MATLAB is expecting to recieve the Torque Parameters
        *(data_to_send_point + 1) = left_leg->Dorsi_Setpoint_Ankle;
        *(data_to_send_point + 2) = right_leg->Setpoint_Ankle;
        *(data_to_send_point + 3) = right_leg->Dorsi_Setpoint_Ankle;
        send_command_message('D', data_to_send_point, 4);
      }
      if (Flag_Knee_Cfg == true) {
        *(data_to_send_point) = left_leg->Setpoint_Knee;      //MATLAB is expecting to recieve the Torque Parameters
        *(data_to_send_point + 1) = left_leg->Flexion_Setpoint_Knee;
        *(data_to_send_point + 2) = right_leg->Setpoint_Knee;
        *(data_to_send_point + 3) = right_leg->Flexion_Setpoint_Knee;
        send_command_message('D', data_to_send_point, 4);
      }

      break;

    case 'd':

      break;




    case 'F':
      receiveVals(32);
      // TN 7/3/19
      if (Flag_Ankle_Cfg == true) {

        left_leg->Previous_Setpoint_Ankle = left_leg->Setpoint_Ankle;
        left_leg->Previous_Dorsi_Setpoint_Ankle = left_leg->Dorsi_Setpoint_Ankle;
        right_leg->Previous_Setpoint_Ankle = right_leg->Setpoint_Ankle;
        right_leg->Previous_Dorsi_Setpoint_Ankle = right_leg->Dorsi_Setpoint_Ankle;
        memcpy(&left_leg->Setpoint_Ankle, holdOnPoint, 8);                         //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
        memcpy(&left_leg->Dorsi_Setpoint_Ankle, holdOnPoint + 8, 8);
        memcpy(&right_leg->Setpoint_Ankle, holdOnPoint + 16, 8);
        memcpy(&right_leg->Dorsi_Setpoint_Ankle, holdOnPoint + 24, 8);

        if (left_leg->Setpoint_Ankle < 0) {
          left_leg->Setpoint_Ankle = 0;
          left_leg->Previous_Setpoint_Ankle = 0;
          left_leg->Dorsi_Setpoint_Ankle = 0;
          left_leg->Previous_Dorsi_Setpoint_Ankle = 0;
          left_leg->coef_in_3_steps_Ankle = 0;
          left_leg->activate_in_3_steps_Ankle = 1;
          left_leg->first_step_Ankle = 1;
          left_leg->num_3_steps_Ankle = 0;
          left_leg->start_step_Ankle = 0;
        } else {
          left_leg->Setpoint_Ankle = abs(left_leg->Setpoint_Ankle);                     //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
          left_leg->Dorsi_Setpoint_Ankle = -abs(left_leg->Dorsi_Setpoint_Ankle);
          //Recieved the large data chunk chopped into bytes, a roundabout way was needed
          left_leg->Previous_Setpoint_Ankle_Pctrl = left_leg->Previous_Setpoint_Ankle;
          left_leg->Setpoint_Ankle_Pctrl = left_leg->Setpoint_Ankle;
          left_leg->activate_in_3_steps_Ankle = 1;
          left_leg->num_3_steps_Ankle = 0;
          left_leg->first_step_Ankle = 1;
          left_leg->start_step_Ankle = 0;
        }

        if (right_leg->Setpoint_Ankle < 0) {
          right_leg->Setpoint_Ankle = 0;
          right_leg->Dorsi_Setpoint_Ankle = 0;
          right_leg->Previous_Setpoint_Ankle = 0;
          right_leg->Previous_Dorsi_Setpoint_Ankle = 0;
          right_leg->coef_in_3_steps_Ankle = 0;
          right_leg->activate_in_3_steps_Ankle = 1;
          right_leg->first_step_Ankle = 1;
          right_leg->num_3_steps_Ankle = 0;
          right_leg->start_step_Ankle = 0;
          Serial.println("Right Setpoint Negative, going to zero");
        } else {
          right_leg->Setpoint_Ankle = -abs(right_leg->Setpoint_Ankle);                    //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
          right_leg->Dorsi_Setpoint_Ankle = abs(right_leg->Dorsi_Setpoint_Ankle);
          //Recieved the large data chunk chopped into bytes, a roundabout way was needed
          right_leg->Setpoint_Ankle_Pctrl = right_leg->Setpoint_Ankle;
          right_leg->Previous_Setpoint_Ankle_Pctrl = right_leg->Previous_Setpoint_Ankle;
          right_leg->activate_in_3_steps_Ankle = 1;
          right_leg->num_3_steps_Ankle = 0;
          right_leg->first_step_Ankle = 1;
          right_leg->start_step_Ankle = 0;
        }
      }
      if (Flag_Knee_Cfg == true) {
        //MATLAB is only sending 1 value, a double, which is 8 bytes
        left_leg->Previous_Setpoint_Knee = left_leg->Setpoint_Knee;
        left_leg->Previous_Flexion_Setpoint_Knee = left_leg->Flexion_Setpoint_Knee;
        right_leg->Previous_Setpoint_Knee = right_leg->Setpoint_Knee;
        right_leg->Previous_Flexion_Setpoint_Knee = right_leg->Flexion_Setpoint_Knee;
        memcpy(&left_leg->Setpoint_Knee, holdOnPoint, 8);                         //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
        memcpy(&left_leg->Flexion_Setpoint_Knee, holdOnPoint + 8, 8);
        memcpy(&right_leg->Setpoint_Knee, holdOnPoint + 16, 8);
        memcpy(&right_leg->Flexion_Setpoint_Knee, holdOnPoint + 24, 8);

        if (left_leg->Setpoint_Knee < 0) {
          left_leg->Setpoint_Knee = 0;
          left_leg->Previous_Setpoint_Knee = 0;
          left_leg->Flexion_Setpoint_Knee = 0;
          left_leg->Previous_Flexion_Setpoint_Knee = 0;
          left_leg->coef_in_3_steps_Knee  = 0;
          left_leg->activate_in_3_steps_Knee  = 1;
          left_leg->first_step_Knee  = 1;
          left_leg->num_3_steps_Knee  = 0;
          left_leg->start_step_Knee  = 0;
        } else {
          left_leg->Setpoint_Knee = abs(left_leg->Setpoint_Knee);
          left_leg->Flexion_Setpoint_Knee = -abs(left_leg->Flexion_Setpoint_Knee);
          left_leg->Previous_Setpoint_Knee_Pctrl = left_leg->Previous_Setpoint_Knee;
          left_leg->Setpoint_Knee_Pctrl = left_leg->Setpoint_Knee;
          left_leg->activate_in_3_steps_Knee  = 1;
          left_leg->num_3_steps_Knee  = 0;
          left_leg->first_step_Knee  = 1;
          left_leg->start_step_Knee  = 0;
        }

        if (right_leg->Setpoint_Knee < 0) {
          right_leg->Setpoint_Knee = 0;
          right_leg->Flexion_Setpoint_Knee = 0;
          right_leg->Previous_Setpoint_Knee = 0;
          right_leg->Previous_Flexion_Setpoint_Knee = 0;
          right_leg->coef_in_3_steps_Knee = 0;
          right_leg->activate_in_3_steps_Knee = 1;
          right_leg->first_step_Knee = 1;
          right_leg->num_3_steps_Knee = 0;
          right_leg->start_step_Knee = 0;
          Serial.println("Right Knee Setpoint Negative, going to zero");
        } else {
          right_leg->Setpoint_Knee = -abs(right_leg->Setpoint_Knee);                    //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
          right_leg->Flexion_Setpoint_Knee = abs(right_leg->Flexion_Setpoint_Knee);
          //Recieved the large data chunk chopped into bytes, a roundabout way was needed
          right_leg->Setpoint_Knee_Pctrl = right_leg->Setpoint_Knee;
          right_leg->Previous_Setpoint_Knee_Pctrl = right_leg->Previous_Setpoint_Knee;
          right_leg->activate_in_3_steps_Knee = 1;
          right_leg->num_3_steps_Knee = 0;
          right_leg->first_step_Knee = 1;
          right_leg->start_step_Knee = 0;
        }
      }
      break;

    case 'f':

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
      torque_calibration();
      write_torque_bias(left_leg->torque_address, left_leg->torque_calibration_value_Ankle);
      write_torque_bias(left_leg->torque_address + sizeof(double) + 1, left_leg->torque_calibration_value_Knee);  // TN 7/25/19
      write_torque_bias(right_leg->torque_address, right_leg->torque_calibration_value_Ankle);
      write_torque_bias(right_leg->torque_address + sizeof(double) + 1, right_leg->torque_calibration_value_Knee);   // TN 7/25/19
      break;

    case 'K':
      // TN 5/9/19
      if (Flag_Ankle_Cfg) {
        *(data_to_send_point) = left_leg->kp;
        *(data_to_send_point + 1) = left_leg->kd;
        *(data_to_send_point + 2) = left_leg->ki;
        *(data_to_send_point + 3) = right_leg->kp;
        *(data_to_send_point + 4) = right_leg->kd;
        *(data_to_send_point + 5) = right_leg->ki;
        send_command_message('K', data_to_send_point, 6);     //MATLAB is expecting to recieve the Torque Parameters
      }
      if (Flag_Knee_Cfg) {
        *(data_to_send_point) = left_leg->kp_Knee;
        *(data_to_send_point + 1) = left_leg->kd_Knee;
        *(data_to_send_point + 2) = left_leg->ki_Knee;
        *(data_to_send_point + 3) = right_leg->kp_Knee;
        *(data_to_send_point + 4) = right_leg->kd_Knee;
        *(data_to_send_point + 5) = right_leg->ki_Knee;
        send_command_message('K', data_to_send_point, 6);     //MATLAB is expecting to recieve the Torque Parameters
      }
      break;

    case 'k':

      break;

    case 'L':

      FSR_CAL_FLAG = 1;

      break;

    case 'M':
      // TN 5/9/19
      if (Flag_Ankle_Cfg) {
        receiveVals(48);                                                //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
        //MATLAB Sent Kp, then Kd, then Ki.
        memcpy(&left_leg->kp, holdOnPoint, 8);                                  //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
        memcpy(&left_leg->kd, holdOnPoint + 8, 8);                              //memory space pointed to by the variable kp_L.  Essentially a roundabout way to change a variable value, but since the bluetooth
        memcpy(&left_leg->ki, holdOnPoint + 16, 8);                             //Recieved the large data chunk chopped into bytes, a roundabout way was needed
        memcpy(&right_leg->kp, holdOnPoint + 24, 8);                                //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
        memcpy(&right_leg->kd, holdOnPoint + 32, 8);                              //memory space pointed to by the variable kp_R.  Essentially a roundabout way to change a variable value, but since the bluetooth
        memcpy(&right_leg->ki, holdOnPoint + 40, 8);                             //Recieved the large data chunk chopped into bytes, a roundabout way was needed
        left_leg->pid.SetTunings(left_leg->kp, left_leg->ki, left_leg->kd);
        right_leg->pid.SetTunings(right_leg->kp, right_leg->ki, right_leg->kd);
      }
      if (Flag_Knee_Cfg) {
        receiveVals(48);                                                //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
        //MATLAB Sent Kp, then Kd, then Ki.
        memcpy(&left_leg->kp_Knee, holdOnPoint, 8);                                  //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
        memcpy(&left_leg->kd_Knee, holdOnPoint + 8, 8);                              //memory space pointed to by the variable kp_L.  Essentially a roundabout way to change a variable value, but since the bluetooth
        memcpy(&left_leg->ki_Knee, holdOnPoint + 16, 8);                             //Recieved the large data chunk chopped into bytes, a roundabout way was needed
        memcpy(&right_leg->kp_Knee, holdOnPoint + 24, 8);                                //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
        memcpy(&right_leg->kd_Knee, holdOnPoint + 32, 8);                              //memory space pointed to by the variable kp_R.  Essentially a roundabout way to change a variable value, but since the bluetooth
        memcpy(&right_leg->ki_Knee, holdOnPoint + 40, 8);                             //Recieved the large data chunk chopped into bytes, a roundabout way was needed
        left_leg->pid_Knee.SetTunings(left_leg->kp_Knee, left_leg->ki_Knee, left_leg->kd_Knee);   // TN 5/13/19
        right_leg->pid_Knee.SetTunings(right_leg->kp_Knee, right_leg->ki_Knee, right_leg->kd_Knee);   // TN 5/13/19
      }
      break;

    case 'm':

      break;

    case 'N':
      *(data_to_send_point) = 0;
      *(data_to_send_point + 1) = 1;
      *(data_to_send_point + 2) = 2;
      send_command_message('N', data_to_send_point, 3);   //For the Arduino to prove to MATLAB that it is behaving, it will send back the character B
      break;

    case '<':
      if ((check_torque_bias(left_leg->torque_address)) && (check_torque_bias(right_leg->torque_address)) &&
          ((check_torque_bias(left_leg->torque_address + sizeof(double) + 1))) && (check_torque_bias(right_leg->torque_address + sizeof(double) + 1)))  // Tn 7/25/19
      {
        left_leg->torque_calibration_value_Ankle = read_torque_bias(left_leg->torque_address);
        left_leg->torque_calibration_value_Knee = read_torque_bias(left_leg->torque_address + sizeof(double) + 1);  // TN 7/25/19
        right_leg->torque_calibration_value_Ankle = read_torque_bias(right_leg->torque_address);
        right_leg->torque_calibration_value_Knee = read_torque_bias(right_leg->torque_address + sizeof(double) + 1); // TN 7/25/19
        left_leg->Tarray[3] = {0};
        right_leg->Tarray[3] = {0};
        left_leg->Tarray_Knee[3] = {0};  // TN 9/3/19
        right_leg->Tarray_Knee[3] = {0};  // TN 9/3/19
        *(data_to_send_point) = 1;
      }
      else
      {
        *(data_to_send_point) = 0;
      }
      if (((check_FSR_values(left_leg->address_FSR)) && (check_FSR_values(left_leg->address_FSR + sizeof(double) + 1))) &&
          ((check_FSR_values(right_leg->address_FSR)) && (check_FSR_values(right_leg->address_FSR + sizeof(double) + 1))))
      {
        if (FLAG_TOE_HEEL_SENSORS) { // TN 7/25/19
          left_leg->fsr_Toe_peak_ref = read_FSR_values(left_leg->address_FSR);
          right_leg->fsr_Toe_peak_ref = read_FSR_values(right_leg->address_FSR);
          left_leg->fsr_Heel_peak_ref = read_FSR_values(left_leg->address_FSR + sizeof(double) + 1);
          right_leg->fsr_Heel_peak_ref = read_FSR_values(right_leg->address_FSR + sizeof(double) + 1);
          left_leg->fsr_Combined_peak_ref = left_leg->fsr_Toe_peak_ref + left_leg->fsr_Heel_peak_ref;
          right_leg->fsr_Combined_peak_ref = right_leg->fsr_Toe_peak_ref + right_leg->fsr_Heel_peak_ref;
        } else {
          left_leg->fsr_Toe_peak_ref = read_FSR_values(left_leg->address_FSR);
          right_leg->fsr_Toe_peak_ref = read_FSR_values(right_leg->address_FSR);
        }

        *(data_to_send_point + 1) = 1;

      }
      else
      {
        *(data_to_send_point + 1) = 0;
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

      // add baseline
      // TN 5/9/19

      if (FLAG_TOE_HEEL_SENSORS) {
        left_leg->p_steps->plant_peak_mean_Toe = read_baseline(left_leg->baseline_address);
        right_leg->p_steps->plant_peak_mean_Toe = read_baseline(right_leg->baseline_address);
        left_leg->baseline_value_Toe = left_leg->p_steps->plant_peak_mean_Toe;
        right_leg->baseline_value_Toe = right_leg->p_steps->plant_peak_mean_Toe;
        left_leg->baseline_value_Ankle = left_leg->baseline_value_Toe;
        right_leg->baseline_value_Ankle = right_leg->baseline_value_Toe;
        
        left_leg->p_steps->plant_peak_mean_HeelMinusToe = read_baseline(left_leg->baseline_address + sizeof(double) + 1);
        right_leg->p_steps->plant_peak_mean_HeelMinusToe = read_baseline(right_leg->baseline_address + sizeof(double) + 1);
        left_leg->p_steps->plant_peak_mean_Heel = read_baseline(left_leg->baseline_address + sizeof(double) + 1);
        right_leg->p_steps->plant_peak_mean_Heel = read_baseline(right_leg->baseline_address + sizeof(double) + 1);
        left_leg->baseline_value_Heel = left_leg->p_steps->plant_peak_mean_Heel;
        right_leg->baseline_value_Heel = right_leg->p_steps->plant_peak_mean_Heel;
        left_leg->baseline_value_HeelMinusToe = left_leg->p_steps->plant_peak_mean_HeelMinusToe;
        right_leg->baseline_value_HeelMinusToe = right_leg->p_steps->plant_peak_mean_HeelMinusToe;

        if(flag_id_KneeHeel){
          right_leg->baseline_value_Knee = right_leg->baseline_value_Heel;
          left_leg->baseline_value_Knee = left_leg->baseline_value_Heel;
        }else{
          right_leg->baseline_value_Knee = right_leg->baseline_value_HeelMinusToe;
          left_leg->baseline_value_Knee = left_leg->baseline_value_HeelMinusToe;
        }
        
      } else if (FLAG_TOE_SENSOR) {
        left_leg->p_steps->plant_peak_mean = read_baseline(left_leg->baseline_address);
        right_leg->p_steps->plant_peak_mean = read_baseline(right_leg->baseline_address);
        left_leg->baseline_value = left_leg->p_steps->plant_peak_mean;
        right_leg->baseline_value = right_leg->p_steps->plant_peak_mean;
      }


      break;

    case '>':
      //------------------------------------------
      if (clean_torque_bias(left_leg->torque_address))
      {
      }
      else
      {
      }
      if (clean_torque_bias(left_leg->torque_address + sizeof(double) + sizeof(char)))
      {
      }
      else
      {
      }
      if (clean_FSR_values(left_leg->address_FSR))
      {
      }
      else
      {
      }
      //------------------------------------------
      if (clean_torque_bias(right_leg->torque_address))
      {
      }
      else
      {
      }
      if (clean_torque_bias(right_leg->torque_address + sizeof(double) + sizeof(char)))
      {
      }
      else
      {
      }
      if (clean_FSR_values(right_leg->address_FSR))
      {
      }
      else
      {
      }
      //------------------------------------------
      if (clean_FSR_values(left_leg->address_FSR + sizeof(double) + sizeof(char)))
      {
      }
      else
      {
      }
      if (clean_FSR_values(right_leg->address_FSR + sizeof(double) + sizeof(char)))
      {
      }
      else
      {
      }
      if (clean_EXP_Parameters(address_params))
      {
      }
      else
      {
      }
      break;

    case '_':
      // TN 7/3/19
      if (Flag_Ankle_Cfg) {
        receiveVals(16);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
        memcpy(&left_leg->KF, holdOnPoint, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
        memcpy(&right_leg->KF, holdOnPoint + 8, 8);
      }
      if (Flag_Knee_Cfg) {
        receiveVals(16);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
        memcpy(&left_leg->KF_Knee, holdOnPoint, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
        memcpy(&right_leg->KF_Knee, holdOnPoint + 8, 8);
      }
      break;

    case '-':

      break;

    case'`':   // TN 5/9/19
      if (Flag_Ankle_Cfg) {
        *(data_to_send_point) = left_leg->KF;
        *(data_to_send_point + 1) = right_leg->KF;
        send_command_message('`', data_to_send_point, 2);     //MATLAB is expecting to recieve the Torque Parameters
      }
      if (Flag_Knee_Cfg) {
        *(data_to_send_point) = left_leg->KF_Knee;
        *(data_to_send_point + 1) = right_leg->KF_Knee;
        send_command_message('`', data_to_send_point, 2);     //MATLAB is expecting to recieve the Torque Parameters
      }
      break;

    case'~':   // TN 5/9/19

      break;

    case')':
      receiveVals(24);                               //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
      //MATLAB Sent N1 N2 and then N3 Paramenters for smoothing (see change pid setpoint)
      memcpy(&N1, holdOnPoint, 8);                   //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      memcpy(&N2, holdOnPoint + 8, 8);               //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
      memcpy(&N3, holdOnPoint + 16, 8);              //Recieved the large data chunk chopped into bytes, a roundabout way was needed

      right_leg->N1 = N1;
      right_leg->N2 = N2;
      right_leg->N3 = N3;

      left_leg->N1 = N1;
      left_leg->N2 = N2;
      left_leg->N3 = N3;

      left_leg->old_N3 = left_leg->N3;
      left_leg->old_N2 = left_leg->N2;
      left_leg->old_N1 = left_leg->N1;

      right_leg->old_N3 = right_leg->N3;
      right_leg->old_N2 = right_leg->N2;
      right_leg->old_N1 = right_leg->N1;

      break;

    case '(':
      *(data_to_send_point) = N1;
      *(data_to_send_point + 1) = N2;
      *(data_to_send_point + 2) = N3;
      send_command_message('(', data_to_send_point, 3);     //MATLAB is expecting to recieve the Torque Parameters

      break;

    case 'P': //GO 5/13/19
      left_leg->torque_calibration_value_Ankle = read_torque_bias(left_leg->torque_address);
      right_leg->torque_calibration_value_Ankle = read_torque_bias(right_leg->torque_address);
      left_leg->torque_calibration_value_Knee = read_torque_bias(left_leg->torque_address + sizeof(double) + 1); // TN 7/25/19
      right_leg->torque_calibration_value_Knee = read_torque_bias(right_leg->torque_address + sizeof(double) + 1); // TN 7/25/19
      break;

    case 'p': //GO 5/13/19
      write_torque_bias(left_leg->torque_address, left_leg->torque_calibration_value_Ankle);
      write_torque_bias(right_leg->torque_address, right_leg->torque_calibration_value_Ankle);
      write_torque_bias(left_leg->torque_address + sizeof(double) + 1, left_leg->torque_calibration_value_Knee); // TN 7/25/19
      write_torque_bias(right_leg->torque_address + sizeof(double) + 1, right_leg->torque_calibration_value_Knee); // TN 7/25/19
      break;



    case 'O':
      break;


    case 'o':
      Old_Control_Mode = Control_Mode;
      Control_Mode = 100;

      right_leg->p_steps->torque_adj = false;
      left_leg->p_steps->torque_adj = false;
      *right_leg->p_Setpoint_Ankle = right_leg->p_steps->Setpoint_A;
      *left_leg->p_Setpoint_Ankle = left_leg->p_steps->Setpoint_A;
      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint_A;
      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint_A;

      *right_leg->p_Setpoint_Knee = right_leg->p_steps->Setpoint_K;// SS 1/15/2020
      *left_leg->p_Setpoint_Knee = left_leg->p_steps->Setpoint_K;// SS 1/15/2020
      *right_leg->p_Setpoint_Knee_Pctrl = right_leg->p_steps->Setpoint_K;// SS 1/15/2020
      *left_leg->p_Setpoint_Knee_Pctrl = left_leg->p_steps->Setpoint_K; // SS 1/15/2020
      break;

    case 'Q':  // TN 7/3/19
      if (Flag_Ankle_Cfg == true) {
        *(data_to_send_point) = left_leg->fsr_percent_thresh_Toe;
        *(data_to_send_point + 1) = right_leg->fsr_percent_thresh_Toe;
        send_command_message('Q', data_to_send_point, 2);     //MATLAB is expecting to recieve the Torque Parameters
      }
      if (Flag_Knee_Cfg == true) {
        *(data_to_send_point) = left_leg->fsr_percent_thresh_Heel;
        *(data_to_send_point + 1) = right_leg->fsr_percent_thresh_Heel;
        send_command_message('Q', data_to_send_point, 2);     //MATLAB is expecting to recieve the Torque Parameters
      }
      break;

    case 'q':  // TN 5/9/19

      break;

    case 'R':   // TN 5/9/19
      if (Flag_Ankle_Cfg == true) {
        receiveVals(16);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
        memcpy(&left_leg->fsr_percent_thresh_Toe, holdOnPoint, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
        memcpy(&right_leg->fsr_percent_thresh_Toe, holdOnPoint + 8, 8);
        left_leg->p_steps->fsr_percent_thresh_Toe = left_leg->fsr_percent_thresh_Toe;
        right_leg->p_steps->fsr_percent_thresh_Toe = right_leg->fsr_percent_thresh_Toe;
      }
      if (Flag_Knee_Cfg == true) {
        receiveVals(16);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
        memcpy(&left_leg->fsr_percent_thresh_Heel, holdOnPoint, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
        memcpy(&right_leg->fsr_percent_thresh_Heel, holdOnPoint + 8, 8);
        left_leg->p_steps->fsr_percent_thresh_Heel = left_leg->fsr_percent_thresh_Heel;
        right_leg->p_steps->fsr_percent_thresh_Heel = right_leg->fsr_percent_thresh_Heel;
      }
      break;

    case 'r':   // TN 5/9/19

      break;

    case 'S':   // TN 7/3/19

      break;


//    case 's':  // TN 7/3/19
//
//      break;

    case 'C':
      while (bluetooth.available() > 0) bluetooth.read();
      break;

    case 'T':
      left_leg->p_steps->count_plant = 0;
      left_leg->p_steps->n_steps = 0;
      left_leg->p_steps->flag_start_plant = false;
      left_leg->p_steps->flag_take_average = false;
      left_leg->p_steps->flag_N3_adjustment_time = false;
      left_leg->p_steps->flag_take_baseline = false;
      left_leg->p_steps->torque_adj = false;
      left_leg->N3 = N3;
      break;

    case 't':
      right_leg->p_steps->count_plant = 0;
      right_leg->p_steps->n_steps = 0;
      right_leg->p_steps->flag_start_plant = false;
      right_leg->p_steps->flag_take_average = false;
      right_leg->p_steps->flag_N3_adjustment_time = false;
      right_leg->p_steps->flag_take_baseline = false;
      right_leg->p_steps->torque_adj = false;
      right_leg->N3 = N3;
      break;

    case 'I':  // TN 7/3/19
      if (Flag_Ankle_Cfg == true){
        *left_leg->p_Setpoint_Ankle = left_leg->p_steps->Setpoint_A;
        *right_leg->p_Setpoint_Ankle = right_leg->p_steps->Setpoint_A;
      }
      if (Flag_Knee_Cfg == true){
        *left_leg->p_Setpoint_Knee = left_leg->p_steps->Setpoint_K;
        *right_leg->p_Setpoint_Knee = right_leg->p_steps->Setpoint_K;
      }
      right_leg->p_steps->torque_adj = false;
      left_leg->p_steps->torque_adj = false;
      break;

    case 'i':
      break;

    case '!':  // TN 5/9/19
      if (stream == 1) {
      } else {
        if (FLAG_TOE_HEEL_SENSORS ) {
          write_FSR_values(left_leg->address_FSR, left_leg->fsr_Toe_peak_ref);
          write_FSR_values(left_leg->address_FSR + sizeof(double) + sizeof(char), left_leg->fsr_Heel_peak_ref);
          write_FSR_values(right_leg->address_FSR, right_leg->fsr_Toe_peak_ref);
          write_FSR_values(right_leg->address_FSR + sizeof(double) + sizeof(char), right_leg->fsr_Heel_peak_ref);
        }
        else
        {
          write_FSR_values(left_leg->address_FSR, left_leg->fsr_Combined_peak_ref / 2);
          write_FSR_values(left_leg->address_FSR + sizeof(double) + sizeof(char), left_leg->fsr_Combined_peak_ref / 2);
          write_FSR_values(right_leg->address_FSR, right_leg->fsr_Combined_peak_ref / 2);
          write_FSR_values(right_leg->address_FSR + sizeof(double) + sizeof(char), right_leg->fsr_Combined_peak_ref / 2);
        }

        // TN 7/25/19
        write_baseline(left_leg->baseline_address, left_leg->baseline_value_Ankle);
        write_baseline(right_leg->baseline_address, right_leg->baseline_value_Ankle);

        write_baseline(left_leg->baseline_address + sizeof(double) + sizeof(char), left_leg->baseline_value_Knee);
        write_baseline(right_leg->baseline_address + sizeof(double) + sizeof(char), right_leg->baseline_value_Knee);
//        write_baseline(left_leg->baseline_address + sizeof(double) + sizeof(char), left_leg->baseline_value_HeelMinusToe);  // SS 9/10/2019
//        write_baseline(right_leg->baseline_address + sizeof(double) + sizeof(char), right_leg->baseline_value_HeelMinusToe);  // SS 9/10/2019


        write_torque_bias(left_leg->torque_address, left_leg->torque_calibration_value_Ankle);
        write_torque_bias(left_leg->torque_address + sizeof(double) + sizeof(char), left_leg->torque_calibration_value_Knee);   // TN 9/3/19
        write_torque_bias(right_leg->torque_address, right_leg->torque_calibration_value_Ankle);                                // TN 9/3/19
        write_torque_bias(right_leg->torque_address + sizeof(double) + sizeof(char), right_leg->torque_calibration_value_Knee);
      }//end if
      break;

    case 'W':
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&app, &holdon, 8);
      left_leg->zero = zero + app;
      break;

    case 'X':
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&app, &holdon, 8);
      right_leg->zero = zero + app;
      break;


    case 'w':
      right_leg->sign = -1;
      break;

    case 'x':
      right_leg->sign = 1;
      break;

    case '[': // Receive Right Gain from GUI   // TN 7/3/19
      Flag_Ankle_Cfg = true;
      Flag_Knee_Cfg = false;
      break;

    case ']': // TN 7/3/19
      Flag_Knee_Cfg = true;
      Flag_Ankle_Cfg = false;
      break;

    case '{': // Receive Left Gain from GUI  // TN 5/9/19
      receiveVals(16);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      if (Flag_Ankle_Cfg == true) {
        memcpy(&left_leg->Prop_Gain, holdOnPoint, 8);
        memcpy(&right_leg->Prop_Gain, holdOnPoint + 8, 8);
      }
      if (Flag_Knee_Cfg == true) {
        memcpy(&left_leg->Prop_Gain_Knee, holdOnPoint, 8);
        memcpy(&right_leg->Prop_Gain_Knee, holdOnPoint + 8, 8);
      }
      break;

    case '}': // Send Left Gain to GUI  // TN 5/9/19

      if (Flag_Ankle_Cfg == true) {
        *(data_to_send_point) = left_leg->Prop_Gain;
        *(data_to_send_point + 1) = right_leg->Prop_Gain;
      }
      if (Flag_Knee_Cfg == true) {
        *(data_to_send_point) = left_leg->Prop_Gain_Knee;
        *(data_to_send_point + 1) = right_leg->Prop_Gain_Knee;
      }
      send_command_message('}', data_to_send_point, 2);     //MATLAB is expecting to recieve the Torque Parameters
      break;

    case '+':
      // TN 5/8/19
      OLD_FLAG_TOE_HEEL_SENSORS = FLAG_TOE_HEEL_SENSORS;
      FLAG_TOE_HEEL_SENSORS = false;
      FLAG_BALANCE = true;
      Old_Control_Mode = Control_Mode;
      Control_Mode = 2;
      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint_A;
      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint_A;
      *right_leg->p_Setpoint_Knee_Pctrl = right_leg->p_steps->Setpoint_K;   // TN 5/9/19
      *left_leg->p_Setpoint_Knee_Pctrl = left_leg->p_steps->Setpoint_K;    // TN 5/9/19
      FLAG_BALANCE = true;
      break;

    case '=':
      // TN 5/8/19
      OLD_FLAG_TOE_HEEL_SENSORS = FLAG_TOE_HEEL_SENSORS;
      FLAG_TOE_HEEL_SENSORS = true;
      FLAG_BALANCE = false;
      Control_Mode = Old_Control_Mode;
      right_leg->p_steps->torque_adj = false;
      left_leg->p_steps->torque_adj = false;
      *right_leg->p_Setpoint_Ankle = right_leg->p_steps->Setpoint_A;
      *left_leg->p_Setpoint_Ankle = left_leg->p_steps->Setpoint_A;
      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint_A;
      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint_A;
      *right_leg->p_Setpoint_Knee = right_leg->p_steps->Setpoint_K;// SS 1/21/2020
      *left_leg->p_Setpoint_Knee = left_leg->p_steps->Setpoint_K;// SS 1/21/2020
      *right_leg->p_Setpoint_Knee_Pctrl = right_leg->p_steps->Setpoint_K; // TN 5/9/19
      *left_leg->p_Setpoint_Knee_Pctrl = left_leg->p_steps->Setpoint_K;// TN 5/9/19
      FLAG_BALANCE = false;
      break;


    case '.':  // TN 7/3/19
      flag_auto_KF = 1;
      if (Flag_Ankle_Cfg == true) {
        left_leg->KF = 1;
        right_leg->KF = 1;
      }
      if (Flag_Knee_Cfg == true) {
        left_leg->KF_Knee = 1;
        right_leg->KF_Knee = 1;
      }
      left_leg->ERR = 0;
      right_leg->ERR = 0;
      left_leg->ERR_Knee = 0;   //  TN 8/30/19
      right_leg->ERR_Knee = 0;   //  TN 8/30/19
      break;

    case ';':  // TN 5/9/19
      flag_auto_KF = 0;
      if (Flag_Ankle_Cfg == true) {
        left_leg->KF = 1;
        right_leg->KF = 1;
      }
      if (Flag_Knee_Cfg == true) {
        left_leg->KF_Knee = 1;
        right_leg->KF_Knee = 1;
      }

      left_leg->ERR = 0;
      right_leg->ERR = 0;
      left_leg->ERR_Knee = 0;   //  TN 8/30/19
      right_leg->ERR_Knee = 0;   //  TN 8/30/19
      break;

    case '#':
      //      OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR;
      //      FLAG_ONE_TOE_SENSOR = true;
      //      Old_Control_Mode = Control_Mode;
      //      Control_Mode = 3; // activate pivot proportional control
      //      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
      //      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
      OLD_FLAG_TOE_HEEL_SENSORS = FLAG_TOE_HEEL_SENSORS; // TN 5/8/19
      FLAG_TOE_HEEL_SENSORS = true; // TN 5/8/19

      flag_id = false; // TN 04/29/19
      flag_id_KneeHeel = false;  //  SS  2/5/2020
      flag_pivot = true; // TN 04/29/19
      if (Flag_Prop_Ctrl == true) // TN 04/29/19
        Control_Mode = 3;
        
      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint_A; // TN 04/29/19
      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint_A; // TN 04/29/19
      *right_leg->p_Setpoint_Knee_Pctrl = right_leg->p_steps->Setpoint_K;  // TN 5/8/19
      *left_leg->p_Setpoint_Knee_Pctrl = left_leg->p_steps->Setpoint_K;     // TN 5/8/19

      break;

    case 'c':
      // OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR;
      // FLAG_ONE_TOE_SENSOR = true;
      //Old_Control_Mode = Control_Mode;
      //      Control_Mode = 4; // activate Inverse Dynamic proportional control
      //      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
      //      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
      OLD_FLAG_TOE_HEEL_SENSORS = FLAG_TOE_HEEL_SENSORS; // TN 5/8/19
      FLAG_TOE_HEEL_SENSORS = true; // TN 5/8/19

      flag_id = true; // TN 04/29/19
      flag_id_KneeHeel = false;  //  SS  2/5/2020
      flag_pivot = false; // TN 04/29/19
      if (Flag_Prop_Ctrl == true) // TN 04/29/19
        Control_Mode = 4; // TN 04/29/19
        
      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint_A; // TN 04/29/19
      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint_A; // TN 04/29/19
      *right_leg->p_Setpoint_Knee_Pctrl = right_leg->p_steps->Setpoint_K;  // TN 5/8/19
      *left_leg->p_Setpoint_Knee_Pctrl = left_leg->p_steps->Setpoint_K;     // TN 5/8/19

      break;

      case 's':
      // OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR;
      // FLAG_ONE_TOE_SENSOR = true;
      //Old_Control_Mode = Control_Mode;
      //      Control_Mode = 4; // activate Inverse Dynamic proportional control
      //      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
      //      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
      OLD_FLAG_TOE_HEEL_SENSORS = FLAG_TOE_HEEL_SENSORS; // TN 5/8/19
      FLAG_TOE_HEEL_SENSORS = true; // TN 5/8/19

      flag_id = true; // TN 04/29/19
      flag_id_KneeHeel = true;  //  SS  2/5/2020
      flag_pivot = false; // TN 04/29/19
      if (Flag_Prop_Ctrl == true) // TN 04/29/19
        Control_Mode = 4; // TN 04/29/19
        
      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint_A; // TN 04/29/19
      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint_A; // TN 04/29/19
      *right_leg->p_Setpoint_Knee_Pctrl = right_leg->p_steps->Setpoint_K;  // TN 5/8/19
      *left_leg->p_Setpoint_Knee_Pctrl = left_leg->p_steps->Setpoint_K;     // TN 5/8/19

      break;

    case 'l': // TN 04/29/19

      Old_Control_Mode = Control_Mode; // TN 04/29/19
      Flag_Prop_Ctrl = true; // TN 04/29/19
      OLD_FLAG_TOE_HEEL_SENSORS = FLAG_TOE_HEEL_SENSORS; // TN 5/8/19
      FLAG_TOE_HEEL_SENSORS = true; // TN 5/8/19

      if (flag_pivot == true)   // TN 04/29/19
        Control_Mode = 3; // activate pivot PC // TN 04/29/19


      if (flag_id == true || flag_id_KneeHeel == true) // TN 04/29/19
        Control_Mode = 4; // activate ID PC // TN 04/29/19

      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint_A; // TN 04/29/19
      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint_A; // TN 04/29/19
      *right_leg->p_Setpoint_Knee_Pctrl = right_leg->p_steps->Setpoint_K;  // TN 5/8/19
      *left_leg->p_Setpoint_Knee_Pctrl = left_leg->p_steps->Setpoint_K;     // TN 5/8/19

      break;


    case '^':
      Control_Mode = Old_Control_Mode;
      OLD_FLAG_TOE_HEEL_SENSORS = FLAG_TOE_HEEL_SENSORS; // TN 5/8/19
      //FLAG_TOE_HEEL_SENSORS = false; // TN 5/8/19
      right_leg->p_steps->torque_adj = false;
      left_leg->p_steps->torque_adj = false;
      *right_leg->p_Setpoint_Ankle = right_leg->p_steps->Setpoint_A;
      *left_leg->p_Setpoint_Ankle = left_leg->p_steps->Setpoint_A;
      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint_A;
      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint_A;
      *right_leg->p_Setpoint_Knee = right_leg->p_steps->Setpoint_K;   // TN 5/8/19
      *left_leg->p_Setpoint_Knee = left_leg->p_steps->Setpoint_K;    // TN 5/8/19
      *right_leg->p_Setpoint_Knee_Pctrl = right_leg->p_steps->Setpoint_K;    // TN 5/8/19
      *left_leg->p_Setpoint_Knee_Pctrl = left_leg->p_steps->Setpoint_K;     // TN 5/8/19
      Flag_Prop_Ctrl = false; // TN 04/29/19
      flag_id = false;  // TN 5//13/19
      flag_id_KneeHeel = false;   //  SS  2/5/2020
      flag_pivot = false;  // TN 5//13/19
      break;

    case 'B':
      if (FLAG_BIOFEEDBACK == true) {
        *(data_to_send_point) = right_leg->Heel_Strike_baseline;
        send_command_message('B', data_to_send_point, 1);

      } else if (FLAG_BALANCE == true) {

        *(data_to_send_point) = left_leg->FSR_Toe_Steady_Balance_Baseline * left_leg->Steady_multiplier;
        *(data_to_send_point + 1) = left_leg->FSR_Heel_Steady_Balance_Baseline * left_leg->Steady_multiplier;
        *(data_to_send_point + 2) = right_leg->FSR_Toe_Steady_Balance_Baseline * right_leg->Steady_multiplier;
        *(data_to_send_point + 3) = right_leg->FSR_Heel_Steady_Balance_Baseline * right_leg->Steady_multiplier;

        *(data_to_send_point + 4) = left_leg->FSR_Toe_Balance_Baseline * left_leg->Dynamic_multiplier;
        *(data_to_send_point + 5) = left_leg->FSR_Heel_Balance_Baseline * left_leg->Dynamic_multiplier;
        *(data_to_send_point + 6) = right_leg->FSR_Toe_Balance_Baseline * right_leg->Dynamic_multiplier;
        *(data_to_send_point + 7) = right_leg->FSR_Heel_Balance_Baseline * right_leg->Dynamic_multiplier;

        send_command_message('B', data_to_send_point, 8);
        // TN 5/8/19
      } else if (FLAG_TOE_HEEL_SENSORS == true) {
        left_leg->baseline_value = left_leg->p_steps->plant_peak_mean_Toe;
        right_leg->baseline_value = right_leg->p_steps->plant_peak_mean_Toe;
        left_leg->baseline_value_Toe = left_leg->p_steps->plant_peak_mean_Toe;
        right_leg->baseline_value_Toe = right_leg->p_steps->plant_peak_mean_Toe;
        left_leg->baseline_value_Ankle = left_leg->p_steps->plant_peak_mean_Toe;
        right_leg->baseline_value_Ankle = right_leg->p_steps->plant_peak_mean_Toe;
        
        left_leg->baseline_value_Heel = left_leg->p_steps->plant_peak_mean_Heel;
        right_leg->baseline_value_Heel = right_leg->p_steps->plant_peak_mean_Heel;
        left_leg->baseline_value_HeelMinusToe = left_leg->p_steps->plant_peak_mean_HeelMinusToe;  // ss 9/10/2019
        right_leg->baseline_value_HeelMinusToe = right_leg->p_steps->plant_peak_mean_HeelMinusToe;  // ss 9/10/2019
        if(flag_id_KneeHeel){
          left_leg->baseline_value_Knee = left_leg->p_steps->plant_peak_mean_Heel;
          right_leg->baseline_value_Knee = right_leg->p_steps->plant_peak_mean_Heel;
        }else{
          left_leg->baseline_value_Knee = left_leg->p_steps->plant_peak_mean_HeelMinusToe;
          right_leg->baseline_value_Knee = right_leg->p_steps->plant_peak_mean_HeelMinusToe;
        }
        
        
        *(data_to_send_point) = left_leg->baseline_value_Ankle; // ss 2/5/2020
        *(data_to_send_point + 1) = right_leg->baseline_value_Ankle; // ss 2/5/2020
        *(data_to_send_point + 2) = left_leg->baseline_value_Knee; // ss 2/5/2020
        *(data_to_send_point + 3) = right_leg->baseline_value_Knee; // ss 2/5/2020
        send_command_message('B', data_to_send_point, 4);

      }
      else {
        left_leg->baseline_value = left_leg->p_steps->plant_peak_mean;
        right_leg->baseline_value = right_leg->p_steps->plant_peak_mean;
         *(data_to_send_point) = left_leg->baseline_value; // ss 2/5/2020
        *(data_to_send_point + 1) = right_leg->baseline_value; // ss 2/5/2020
        *(data_to_send_point + 2) = left_leg->baseline_value_Knee; // ss 2/5/2020
        *(data_to_send_point + 3) = right_leg->baseline_value_Knee; // ss 2/5/2020
        send_command_message('B', data_to_send_point, 4);
      }


      break;

    case 'b':
      left_leg->FSR_baseline_FLAG = 1;
      right_leg->FSR_baseline_FLAG = 1;
      base_1 = 0;
      base_2 = 0;
      left_leg->p_steps->count_plant_base = 0;
      right_leg->p_steps->count_plant_base = 0;
      right_leg->p_steps->flag_start_plant = false;
      left_leg->p_steps->flag_start_plant = false;
      break;

    case '&':
      FLAG_BALANCE_BASELINE = 1;

      startTime = millis();
      Control_Mode = Old_Control_Mode;// you cannot calibrate if your doing something
      left_leg->FSR_Toe_Balance_Baseline = 0;
      right_leg->FSR_Toe_Balance_Baseline = 0;
      left_leg->FSR_Heel_Balance_Baseline = 0;
      right_leg->FSR_Heel_Balance_Baseline = 0;
      count_balance = 0;
      break;

    case 'J':
      FLAG_STEADY_BALANCE_BASELINE = 1;

      startTime = millis();
      Control_Mode = Old_Control_Mode;// you cannot calibrate if your doing something
      left_leg->FSR_Toe_Steady_Balance_Baseline = 0;
      right_leg->FSR_Toe_Steady_Balance_Baseline = 0;
      left_leg->FSR_Heel_Steady_Balance_Baseline = 0;
      right_leg->FSR_Heel_Steady_Balance_Baseline = 0;
      count_steady_baseline = 0;
      break;


    case '|':
      FLAG_AUTO_RECONNECT_BT = true;
      break;

    case '@':
      FLAG_AUTO_RECONNECT_BT = false;
      break;


    case 'V':
      *(data_to_send_point) = left_leg->Steady_multiplier;
      send_command_message('V', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
      break;

    case 'v':
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->Steady_multiplier, &holdon, 8);
      memcpy(&right_leg->Steady_multiplier, &holdon, 8);
      break;

    case 'A':
      *(data_to_send_point) = left_leg->Dynamic_multiplier;
      send_command_message('A', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
      break;

    case 'a':
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->Dynamic_multiplier, &holdon, 8);
      memcpy(&right_leg->Dynamic_multiplier, &holdon, 8);
      break;


    case '/':
      // TN 5/8/19
      OLD_FLAG_TOE_HEEL_SENSORS = FLAG_TOE_HEEL_SENSORS;
      FLAG_TOE_HEEL_SENSORS = false;
      FLAG_BIOFEEDBACK = true;
      right_leg->BIO_BASELINE_FLAG = false;
      break;


    case 'y':
      // TN 5/8/19
      FLAG_TOE_HEEL_SENSORS = OLD_FLAG_TOE_HEEL_SENSORS;
      FLAG_BIOFEEDBACK = false;
      if (left_leg->state == 2) left_leg->state = 1;
      if (right_leg->state == 2) right_leg->state = 1;
      break;


    case 'u':
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->BioFeedback_desired, &holdon, 8);
      right_leg->BioFeedback_desired = left_leg->BioFeedback_desired;
      break;

    case '*':
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&treadmill_speed, &holdon, 8); //YF
      break;

    // Optimization ------------------------------------------------

    case '%':
      Serial.println("Start Optimization");
      Flag_HLO = true;
      break;

    case 'h':
      //Serial.println("End Optimization");
      left_leg->Setpoint_Ankle = 0;
      right_leg->Setpoint_Ankle = 0;
      left_leg->Setpoint_Ankle_Pctrl = 0;
      right_leg->Setpoint_Ankle_Pctrl = 0;
      left_leg->Dorsi_Setpoint_Ankle = 0;
      right_leg->Dorsi_Setpoint_Ankle = 0;
      Flag_HLO = false;
      break;

    case '$':
      if (Flag_HLO) {
        receiveVals(16);
        memcpy(&left_leg->Setpoint_Ankle_Opt, holdOnPoint, 8); //Ankle torque setpoint for bang-bang
        memcpy(&left_leg->T_Opt_p, holdOnPoint + 8, 8);       //Ankle torque rise time percentage for bang-bang
        right_leg->Setpoint_Ankle_Opt = -left_leg->Setpoint_Ankle_Opt;
        right_leg->T_Opt_p = left_leg->T_Opt_p;
        //Serial.println(left_leg->FLAG_UPDATE_VALUES);
        left_leg->FLAG_UPDATE_VALUES = true;
        right_leg->FLAG_UPDATE_VALUES = true;
        //Serial.println(left_leg->FLAG_UPDATE_VALUES);
        //Serial.print("Received these values from HLO : ");
        //Serial.print(left_leg->Setpoint_Ankle_Opt);
        //Serial.print(" , ");
        //Serial.println(left_leg->T_Opt_p);

        left_leg->activate_in_3_steps_Ankle = 1;
        left_leg->num_3_steps_Ankle = 0;
        left_leg->first_step_Ankle = 1;
        left_leg->start_step_Ankle = 0;

        right_leg->activate_in_3_steps_Ankle = 1;
        right_leg->num_3_steps_Ankle = 0;
        right_leg->first_step_Ankle = 1;
        right_leg->start_step_Ankle = 0;
      }

    case '"':
      if (Flag_HLO) {
        left_leg->Previous_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint_A;
        right_leg->Previous_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint_A;
        left_leg->Previous_Dorsi_Setpoint_Ankle = left_leg->Dorsi_Setpoint_Ankle;
        right_leg->Previous_Dorsi_Setpoint_Ankle = right_leg->Dorsi_Setpoint_Ankle;
        receiveVals(8);
        memcpy(&left_leg->Setpoint_Ankle, holdOnPoint, 8); //HLO proportional control setpoint
        right_leg->Setpoint_Ankle = -left_leg->Setpoint_Ankle;
        left_leg->activate_in_3_steps_Ankle = 1;
        left_leg->num_3_steps_Ankle = 0;
        left_leg->first_step_Ankle = 1;
        left_leg->start_step_Ankle = 0;
        right_leg->activate_in_3_steps_Ankle = 1;
        right_leg->num_3_steps_Ankle = 0;
        right_leg->first_step_Ankle = 1;
        right_leg->start_step_Ankle = 0;
        left_leg->FSR_baseline_FLAG = 1;                      //Retake the baseline every new setpoint
        right_leg->FSR_baseline_FLAG = 1;
        break;
      }



    // ------------------------------------------------------------


    case ':':
      left_leg->BioFeedback_Baseline_flag = false;
      right_leg->BioFeedback_Baseline_flag = false;
      left_leg->BIO_BASELINE_FLAG = true;
      right_leg->BIO_BASELINE_FLAG = true;
      left_leg->Heel_Strike = 0;
      right_leg->Heel_Strike = 0;
      left_leg->Heel_Strike_Count = 0;
      right_leg->Heel_Strike_Count = 0;
      left_leg->score = 0;
      right_leg->score = 0;
      break;

    case 'U':
      data_to_send_point[0] = (double) VERSION;
      data_to_send_point[1] = (double) BOARD_VERSION;
      send_command_message('U', data_to_send_point, 2);
      break;

    case 'z':
      flag_motor_error_check = !flag_motor_error_check;
      data_to_send_point[0] = flag_motor_error_check;
      send_command_message('z', data_to_send_point, 1);
      break;


    case 'e':

      bluetooth.print('S');
      bluetooth.print('P');
      bluetooth.print(',');
      bluetooth.print(left_leg->p_steps->plant_peak_mean_Toe);
      bluetooth.print(',');
      bluetooth.print(right_leg->p_steps->plant_peak_mean_Toe);
      bluetooth.print(',');
      bluetooth.print(left_leg->p_steps->plant_peak_mean_Heel);
      bluetooth.print(',');
      bluetooth.print(right_leg->p_steps->plant_peak_mean_Heel);
      bluetooth.print(',');
      bluetooth.print(left_leg->torque_calibration_value_Ankle);
      bluetooth.print(',');
      bluetooth.print(right_leg->torque_calibration_value_Ankle);
      bluetooth.print(',');
      bluetooth.print(left_leg->torque_calibration_value_Knee);
      bluetooth.print(',');
      bluetooth.print(right_leg->torque_calibration_value_Knee);
      bluetooth.print(',');
      bluetooth.println('Z');





      break;


    case 'g':

      receiveVals(80);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      if (FLAG_TOE_HEEL_SENSORS == true) {
        memcpy(&left_leg->p_steps->plant_peak_mean_temp_Toe, holdOnPoint, 8);
        memcpy(&right_leg->p_steps->plant_peak_mean_temp_Toe, holdOnPoint + 8, 8);//added

        memcpy(&left_leg->p_steps->plant_peak_mean_temp_Heel, holdOnPoint + 16, 8);             // send an old value of plant_peak_mean to teensy  // TN 04-26-2019
        memcpy(&right_leg->p_steps->plant_peak_mean_temp_Heel, holdOnPoint + 24, 8);//added          // send an old value of plant_peak_mean to teensy  // TN 04-26-2019

//         memcpy(&left_leg->p_steps->plant_peak_mean_temp_HeelMinusToe, holdOnPoint + 32, 8);             // send an old value of plant_peak_mean to teensy  // SS 9/10/2019
//        memcpy(&right_leg->p_steps->plant_peak_mean_temp_HeelMinusToe, holdOnPoint + 40, 8);//added          // send an old value of plant_peak_mean to teensy  // SS 9/10/2019
//
//        memcpy(&left_leg->torque_calibration_value_Ankle, holdOnPoint + 48, 8);
//        memcpy(&right_leg->torque_calibration_value_Ankle, holdOnPoint + 56, 8);//added
//        memcpy(&left_leg->torque_calibration_value_Knee, holdOnPoint + 64, 8);               // TN 7/15/19
//        memcpy(&right_leg->torque_calibration_value_Knee, holdOnPoint + 72, 8);//added            // TN 8/28/19
//
        memcpy(&left_leg->torque_calibration_value_Ankle, holdOnPoint + 32, 8);
        memcpy(&right_leg->torque_calibration_value_Ankle, holdOnPoint + 40, 8);//added
        memcpy(&left_leg->torque_calibration_value_Knee, holdOnPoint + 48, 8);               // TN 7/15/19
        memcpy(&right_leg->torque_calibration_value_Knee, holdOnPoint + 56, 8);//added            // TN 8/28/19
      }

      break;

    case 'j':   // TN
      Control_Mode = 5;  // Averaged torque profiles
      Serial.println(Control_Mode);

      Serial.println(right_leg->PID_Setpoint);
      Serial.println(left_leg->PID_Setpoint);

      break;


  }
  cmd_from_Gui = 0;
}
