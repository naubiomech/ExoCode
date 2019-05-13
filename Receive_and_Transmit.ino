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


    case 'D':                                         //if MATLAB sent the character D
      *(data_to_send_point) = left_leg->Setpoint_Ankle;      //MATLAB is expecting to recieve the Torque Parameters
      *(data_to_send_point + 1) = left_leg->Dorsi_Setpoint_Ankle;
      send_command_message('D', data_to_send_point, 2);
      break;

    case 'd':
      *(data_to_send_point) = right_leg->Setpoint_Ankle;
      *(data_to_send_point + 1) = right_leg->Dorsi_Setpoint_Ankle;
      send_command_message('d', data_to_send_point, 2);     //MATLAB is expecting to receive the Torque Parameters
      break;

    case 'F':
      receiveVals(16);                                                 //MATLAB is only sending 1 value, a double, which is 8 bytes
      left_leg->Previous_Setpoint_Ankle = left_leg->Setpoint_Ankle;
      left_leg->Previous_Dorsi_Setpoint_Ankle = left_leg->Dorsi_Setpoint_Ankle;
      memcpy(&left_leg->Setpoint_Ankle, holdOnPoint, 8);                         //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      memcpy(&left_leg->Dorsi_Setpoint_Ankle, holdOnPoint + 8, 8);

      if (left_leg->Setpoint_Ankle < 0) {
        left_leg->Setpoint_Ankle = 0;
        left_leg->Previous_Setpoint_Ankle = 0;
        left_leg->Dorsi_Setpoint_Ankle = 0;
        left_leg->Previous_Dorsi_Setpoint_Ankle = 0;
        left_leg->coef_in_3_steps = 0;
        left_leg->activate_in_3_steps = 1;
        left_leg->first_step = 1;
        left_leg->num_3_steps = 0;
        left_leg->start_step = 0;
      } else {
        left_leg->Setpoint_Ankle = abs(left_leg->Setpoint_Ankle);                     //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
        left_leg->Dorsi_Setpoint_Ankle = -abs(left_leg->Dorsi_Setpoint_Ankle);
        //Recieved the large data chunk chopped into bytes, a roundabout way was needed
        left_leg->Previous_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
        left_leg->p_steps->Setpoint = left_leg->sign * left_leg->Setpoint_Ankle;
        left_leg->Setpoint_Ankle_Pctrl = left_leg->Setpoint_Ankle;
        left_leg->activate_in_3_steps = 1;
        left_leg->num_3_steps = 0;
        left_leg->first_step = 1;
        left_leg->start_step = 0;
      }
      break;

    case 'f':
      receiveVals(16);                                                 //MATLAB is only sending 1 value, a double, which is 8 bytes
      right_leg->Previous_Setpoint_Ankle = right_leg->Setpoint_Ankle;
      right_leg->Previous_Dorsi_Setpoint_Ankle = right_leg->Dorsi_Setpoint_Ankle;
      memcpy(&right_leg->Setpoint_Ankle, holdOnPoint, 8);                         //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      memcpy(&right_leg->Dorsi_Setpoint_Ankle, holdOnPoint + 8, 8);
      if (right_leg->Setpoint_Ankle < 0) {

        right_leg->Setpoint_Ankle = 0;
        right_leg->Dorsi_Setpoint_Ankle = 0;
        right_leg->Previous_Setpoint_Ankle = 0;
        right_leg->Previous_Dorsi_Setpoint_Ankle = 0;
        right_leg->coef_in_3_steps = 0;
        right_leg->activate_in_3_steps = 1;
        right_leg->first_step = 1;
        right_leg->num_3_steps = 0;
        right_leg->start_step = 0;
        Serial.println("Right Setpoint Negative, going to zero");

      } else {

        right_leg->Setpoint_Ankle = -abs(right_leg->Setpoint_Ankle);                    //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
        right_leg->Dorsi_Setpoint_Ankle = abs(right_leg->Dorsi_Setpoint_Ankle);
        //Recieved the large data chunk chopped into bytes, a roundabout way was needed
        right_leg->Setpoint_Ankle_Pctrl = right_leg->Setpoint_Ankle;
        right_leg->Previous_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
        right_leg->p_steps->Setpoint = right_leg->sign * right_leg->Setpoint_Ankle;
        right_leg->activate_in_3_steps = 1;
        right_leg->num_3_steps = 0;
        right_leg->first_step = 1;
        right_leg->start_step = 0;
      }
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
      write_torque_bias(left_leg->torque_address, left_leg->torque_calibration_value);
      write_torque_bias(right_leg->torque_address, right_leg->torque_calibration_value);
      break;

    case 'K':
      *(data_to_send_point) = left_leg->kp;
      *(data_to_send_point + 1) = left_leg->kd;
      *(data_to_send_point + 2) = left_leg->ki;
      send_command_message('K', data_to_send_point, 3);     //MATLAB is expecting to recieve the Torque Parameters
      break;

    case 'k':
      *(data_to_send_point) = right_leg->kp;
      *(data_to_send_point + 1) = right_leg->kd;
      *(data_to_send_point + 2) = right_leg->ki;
      send_command_message('k', data_to_send_point, 3);     //MATLAB is expecting to recieve the Torque Parameters
      break;

    case 'L':

      FSR_CAL_FLAG = 1;

      break;

    case 'M':
      receiveVals(24);                                                //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
      //MATLAB Sent Kp, then Kd, then Ki.
      memcpy(&left_leg->kp, holdOnPoint, 8);                                  //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      memcpy(&left_leg->kd, holdOnPoint + 8, 8);                              //memory space pointed to by the variable kp_L.  Essentially a roundabout way to change a variable value, but since the bluetooth
      memcpy(&left_leg->ki, holdOnPoint + 16, 8);                             //Recieved the large data chunk chopped into bytes, a roundabout way was needed
      left_leg->pid.SetTunings(left_leg->kp, left_leg->ki, left_leg->kd);
      break;

    case 'm':
      receiveVals(24);                                                //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
      //MATLAB Sent Kp, then Kd, then Ki.
      memcpy(&right_leg->kp, holdOnPoint, 8);                                  //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      memcpy(&right_leg->kd, holdOnPoint + 8, 8);                              //memory space pointed to by the variable kp_R.  Essentially a roundabout way to change a variable value, but since the bluetooth
      memcpy(&right_leg->ki, holdOnPoint + 16, 8);                             //Recieved the large data chunk chopped into bytes, a roundabout way was needed
      right_leg->pid.SetTunings(right_leg->kp, right_leg->ki, right_leg->kd);
      break;

    case 'N':
      *(data_to_send_point) = 0;
      *(data_to_send_point + 1) = 1;
      *(data_to_send_point + 2) = 2;
      send_command_message('N', data_to_send_point, 3);   //For the Arduino to prove to MATLAB that it is behaving, it will send back the character B
      break;

    case '<':
      if ((check_torque_bias(left_leg->torque_address)) && (check_torque_bias(right_leg->torque_address)))
      {
        left_leg->torque_calibration_value = read_torque_bias(left_leg->torque_address);
        right_leg->torque_calibration_value = read_torque_bias(right_leg->torque_address);
        left_leg->Tarray[3] = {0};
        right_leg->Tarray[3] = {0};
        *(data_to_send_point) = 1;
      }
      else
      {
        *(data_to_send_point) = 0;
      }
      if (((check_FSR_values(left_leg->address_FSR)) && (check_FSR_values(left_leg->address_FSR + sizeof(double) + 1))) &&
          ((check_FSR_values(right_leg->address_FSR)) && (check_FSR_values(right_leg->address_FSR + sizeof(double) + 1))))
      {
        if (FLAG_ONE_TOE_SENSOR) {
          left_leg->fsr_Combined_peak_ref = read_FSR_values(left_leg->address_FSR) + read_FSR_values(left_leg->address_FSR + sizeof(double) + 1);
          right_leg->fsr_Combined_peak_ref = read_FSR_values(right_leg->address_FSR) + read_FSR_values(right_leg->address_FSR + sizeof(double) + 1);
        } else {
          left_leg->fsr_Toe_peak_ref = read_FSR_values(left_leg->address_FSR);
          right_leg->fsr_Toe_peak_ref = read_FSR_values(right_leg->address_FSR);
          left_leg->fsr_Heel_peak_ref = read_FSR_values(left_leg->address_FSR + sizeof(double) + 1);
          right_leg->fsr_Heel_peak_ref = read_FSR_values(right_leg->address_FSR + sizeof(double) + 1);
        }

        *(data_to_send_point + 1) = 1;
        if (FLAG_ONE_TOE_SENSOR) {
        } else {
        }
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
      left_leg->p_steps->plant_peak_mean = read_baseline(left_leg->baseline_address);
      right_leg->p_steps->plant_peak_mean = read_baseline(right_leg->baseline_address);
      left_leg->baseline_value = left_leg->p_steps->plant_peak_mean;
      right_leg->baseline_value = right_leg->p_steps->plant_peak_mean;
      break;

    case '>':
      //------------------------------------------
      if (clean_torque_bias(left_leg->torque_address))
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
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->KF, &holdon, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      break;

    case '-':
      receiveVals(8);
      memcpy(&right_leg->KF, &holdon, 8);
      break;

    case'`':
      *(data_to_send_point) = left_leg->KF;
      send_command_message('`', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
      break;

    case'~':
      *(data_to_send_point) = right_leg->KF;
      send_command_message('~', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
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

    case 'P':
      break;

    case 'p':
      break;

    case 'O':
      break;


    case 'o':
      break;

    case 'Q':
      *(data_to_send_point) = left_leg->fsr_percent_thresh_Toe;
      send_command_message('Q', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
      break;

    case 'q':
      *(data_to_send_point) = right_leg->fsr_percent_thresh_Toe;
      send_command_message('q', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
      break;

    case 'R':
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->fsr_percent_thresh_Toe, &holdon, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
      left_leg->p_steps->fsr_percent_thresh_Toe = left_leg->fsr_percent_thresh_Toe;
      break;

    case 'r':
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&right_leg->fsr_percent_thresh_Toe, &holdon, 8);
      right_leg->p_steps->fsr_percent_thresh_Toe = right_leg->fsr_percent_thresh_Toe;
      break;

    case 'S':
      break;

    case's':
      break;

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

    case 'I':
      *left_leg->p_Setpoint_Ankle = left_leg->p_steps->Setpoint;
      left_leg->p_steps->torque_adj = false;
      break;

    case 'i':

      *right_leg->p_Setpoint_Ankle = right_leg->p_steps->Setpoint;
      right_leg->p_steps->torque_adj = false;
      break;

    case '!':
      if (stream == 1) {
      } else {
        if (FLAG_ONE_TOE_SENSOR) {
          write_FSR_values(left_leg->address_FSR, left_leg->fsr_Combined_peak_ref / 2);
          write_FSR_values((left_leg->address_FSR + sizeof(double) + sizeof(char)), left_leg->fsr_Combined_peak_ref / 2);
          write_FSR_values(right_leg->address_FSR, right_leg->fsr_Combined_peak_ref / 2);
          write_FSR_values((right_leg->address_FSR + sizeof(double) + sizeof(char)), right_leg->fsr_Combined_peak_ref / 2);
        } else {
          write_FSR_values(left_leg->address_FSR, left_leg->fsr_Toe_peak_ref);
          write_FSR_values((left_leg->address_FSR + sizeof(double) + sizeof(char)), left_leg->fsr_Heel_peak_ref);
          write_FSR_values(right_leg->address_FSR, right_leg->fsr_Toe_peak_ref);
          write_FSR_values((right_leg->address_FSR + sizeof(double) + sizeof(char)), right_leg->fsr_Heel_peak_ref);
        }

        write_baseline(left_leg->baseline_address, left_leg->baseline_value);
        write_baseline(right_leg->baseline_address, right_leg->baseline_value);

        write_torque_bias(left_leg->torque_address, left_leg->torque_calibration_value);
        write_torque_bias(right_leg->torque_address, right_leg->torque_calibration_value);
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

    case '[': // Receive Right Gain from GUI
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&right_leg->Prop_Gain, &holdon, 8);
      break;

    case ']': // Send Right Gain to GUI
      *(data_to_send_point) = right_leg->Prop_Gain;
      send_command_message(']', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
      break;

    case '{': // Receive Left Gain from GUI
      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->Prop_Gain, &holdon, 8);
      break;

    case '}': // Send Left Gain to GUI
      *(data_to_send_point) = left_leg->Prop_Gain;
      send_command_message('}', data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
      break;

    case '+':

      OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR;
      FLAG_ONE_TOE_SENSOR = false;
      FLAG_BALANCE = true;
      Old_Control_Mode = Control_Mode;
      Control_Mode = 2;
      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
      FLAG_BALANCE = true;
      break;

    case '=':
      FLAG_ONE_TOE_SENSOR = OLD_FLAG_ONE_TOE_SENSOR;
      FLAG_ONE_TOE_SENSOR = true;
      FLAG_BALANCE = false;
      Control_Mode = Old_Control_Mode;
      right_leg->p_steps->torque_adj = false;
      left_leg->p_steps->torque_adj = false;
      *right_leg->p_Setpoint_Ankle = right_leg->p_steps->Setpoint;
      *left_leg->p_Setpoint_Ankle = left_leg->p_steps->Setpoint;
      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
      FLAG_BALANCE = false;
      break;


    case '.':
      flag_auto_KF = 1;
      left_leg->KF = 1;
      right_leg->KF = 1;
      left_leg->ERR = 0;
      right_leg->ERR = 0;
      break;

    case ';':
      flag_auto_KF = 0;
      left_leg->KF = 1;
      right_leg->KF = 1;
      left_leg->ERR = 0;
      right_leg->ERR = 0;
      break;

    case '#':
      //      OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR;
      //      FLAG_ONE_TOE_SENSOR = true;
      //      Old_Control_Mode = Control_Mode;
      //      Control_Mode = 3; // activate pivot proportional control
      //      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
      //      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
      flag_id = false; // TN 04/29/19
      flag_pivot = true; // TN 04/29/19
      if (Flag_Prop_Ctrl == true) // TN 04/29/19
        Control_Mode = 3;
      break;

    case 'c':
      // OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR;
      // FLAG_ONE_TOE_SENSOR = true;
      //Old_Control_Mode = Control_Mode;
      //      Control_Mode = 4; // activate Inverse Dynamic proportional control
      //      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
      //      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
      flag_id = true; // TN 04/29/19
      flag_pivot = false; // TN 04/29/19
      if (Flag_Prop_Ctrl == true) // TN 04/29/19
        Control_Mode = 4; // TN 04/29/19
      break;

    case 'l': // TN 04/29/19
      OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR; // TN 04/29/19
      FLAG_ONE_TOE_SENSOR = true; // TN 04/29/19
      Old_Control_Mode = Control_Mode; // TN 04/29/19
      Flag_Prop_Ctrl = true; // TN 04/29/19
      if (flag_pivot == true)   // TN 04/29/19
        Control_Mode = 3; // activate pivot PC // TN 04/29/19
      if (flag_id == true) // TN 04/29/19
        Control_Mode = 4; // activate ID PC // TN 04/29/19
      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint; // TN 04/29/19
      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint; // TN 04/29/19
      break;


    case '^':
      Control_Mode = Old_Control_Mode;
      OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR; //GO 4/23/19
      FLAG_ONE_TOE_SENSOR = false; //GO 4/23/19 to return the control to bang-bang (heel-toe)
      right_leg->p_steps->torque_adj = false;
      left_leg->p_steps->torque_adj = false;
      *right_leg->p_Setpoint_Ankle = right_leg->p_steps->Setpoint;
      *left_leg->p_Setpoint_Ankle = left_leg->p_steps->Setpoint;
      *right_leg->p_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
      *left_leg->p_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
      flag_id = false; // TN 05/13/19
      flag_pivot = false; // TN 05/13/19
      Flag_Prop_Ctrl = false; // TN 04/29/19
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

      }
      else {
        left_leg->baseline_value = left_leg->p_steps->plant_peak_mean;
        right_leg->baseline_value = right_leg->p_steps->plant_peak_mean;
        *(data_to_send_point) = left_leg->p_steps->plant_peak_mean;
        *(data_to_send_point + 1) = right_leg->p_steps->plant_peak_mean;
        send_command_message('B', data_to_send_point, 2);
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
      OLD_FLAG_ONE_TOE_SENSOR = FLAG_ONE_TOE_SENSOR;
      FLAG_ONE_TOE_SENSOR = false;
      FLAG_BIOFEEDBACK = true;
      right_leg->BIO_BASELINE_FLAG = false;
      break;


    case 'y':
      FLAG_ONE_TOE_SENSOR = OLD_FLAG_ONE_TOE_SENSOR;
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

        left_leg->activate_in_3_steps = 1;
        left_leg->num_3_steps = 0;
        left_leg->first_step = 1;
        left_leg->start_step = 0;

        right_leg->activate_in_3_steps = 1;
        right_leg->num_3_steps = 0;
        right_leg->first_step = 1;
        right_leg->start_step = 0;
      }

    case '"':
      if (Flag_HLO) {
        left_leg->Previous_Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
        right_leg->Previous_Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
        left_leg->Previous_Dorsi_Setpoint_Ankle = left_leg->Dorsi_Setpoint_Ankle;
        right_leg->Previous_Dorsi_Setpoint_Ankle = right_leg->Dorsi_Setpoint_Ankle;
        receiveVals(8);
        memcpy(&left_leg->Setpoint_Ankle, holdOnPoint, 8); //HLO proportional control setpoint
        right_leg->Setpoint_Ankle = -left_leg->Setpoint_Ankle;
        left_leg->activate_in_3_steps = 1;
        left_leg->num_3_steps = 0;
        left_leg->first_step = 1;
        left_leg->start_step = 0;
        right_leg->activate_in_3_steps = 1;
        right_leg->num_3_steps = 0;
        right_leg->first_step = 1;
        right_leg->start_step = 0;
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
      bluetooth.print(left_leg->p_steps->plant_peak_mean);
      bluetooth.print(',');
      bluetooth.print(right_leg->p_steps->plant_peak_mean);
      bluetooth.print(',');
      bluetooth.print(left_leg->Curr_Combined);
      bluetooth.print(',');
      bluetooth.print(right_leg->Curr_Combined);
      bluetooth.print(',');
      bluetooth.print(left_leg->fsr_Combined_peak_ref);
      bluetooth.print(',');
      bluetooth.print(right_leg->fsr_Combined_peak_ref);
      bluetooth.print(',');
      bluetooth.print(left_leg->fsr_Toe_peak_ref);
      bluetooth.print(',');
      bluetooth.print(right_leg->fsr_Toe_peak_ref);
      bluetooth.print(',');
      bluetooth.print(left_leg->fsr_Heel_peak_ref);
      bluetooth.print(',');
      bluetooth.print(right_leg->fsr_Heel_peak_ref);
      bluetooth.print(',');
      bluetooth.print(left_leg->torque_calibration_value);
      bluetooth.print(',');
      bluetooth.print(right_leg->torque_calibration_value);
      bluetooth.print(',');
      bluetooth.println('Z');


      //      *(data_to_send_point) = left_leg->p_steps->plant_peak_mean;
      //      *(data_to_send_point + 1) = right_leg->p_steps->plant_peak_mean;
      //      send_command_message('P', data_to_send_point, 2);

      //      Serial.println(left_leg->p_steps->plant_peak_mean);
      //      Serial.println(right_leg->p_steps->plant_peak_mean);
      //      Serial.println(left_leg->Curr_Combined);
      //      Serial.println(right_leg->Curr_Combined);
      //      Serial.println(left_leg->fsr_Combined_peak_ref);
      //      Serial.println(right_leg->fsr_Combined_peak_ref);
      //      Serial.println(left_leg->fsr_Toe_peak_ref);
      //      Serial.println(right_leg->fsr_Toe_peak_ref);
      //      Serial.println(left_leg->fsr_Heel_peak_ref);
      //      Serial.println(right_leg->fsr_Heel_peak_ref);
      //      Serial.println(left_leg->torque_calibration_value);
      //      Serial.println(right_leg->torque_calibration_value);



      break;


    case 'g':

      receiveVals(96);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->p_steps->plant_peak_mean_temp, holdOnPoint, 8);              // send an old value of plant_peak_mean to teensy  // TN 04-26-2019
      //delay(10);
      //receiveVals(8);
      memcpy(&right_leg->p_steps->plant_peak_mean_temp, holdOnPoint + 8, 8);//added          // send an old value of plant_peak_mean to teensy  // TN 04-26-2019
      //delay(10);
      //receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->Curr_Combined, holdOnPoint + 16, 8);
      //      delay(10);
      //      receiveVals(8);
      memcpy(&right_leg->Curr_Combined, holdOnPoint + 24, 8);//added
      //      delay(10);
      //      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->fsr_Combined_peak_ref, holdOnPoint + 32, 8);
      //      delay(10);
      //      receiveVals(8);
      memcpy(&right_leg->fsr_Combined_peak_ref, holdOnPoint + 40, 8);//added
      //      delay(10);
      //      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->fsr_Toe_peak_ref, holdOnPoint + 48, 8);
      //      delay(10);
      //      receiveVals(8);
      memcpy(&right_leg->fsr_Toe_peak_ref, holdOnPoint + 56, 8);//added
      //      delay(10);
      //      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->fsr_Heel_peak_ref, holdOnPoint + 64, 8);
      //      delay(10);
      //      receiveVals(8);
      memcpy(&right_leg->fsr_Heel_peak_ref, holdOnPoint + 72, 8);//added
      //      delay(10);
      //      receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
      memcpy(&left_leg->torque_calibration_value, holdOnPoint + 80, 8);
      //      delay(10);
      //      receiveVals(8);
      memcpy(&right_leg->torque_calibration_value, holdOnPoint + 88, 8);//added


      Serial.println("Old left_leg->p_steps->plant_peak_mean_temp");
      Serial.println(left_leg->p_steps->plant_peak_mean_temp);
      Serial.println("Old right_leg->p_steps->plant_peak_mean_temp");
      Serial.println(right_leg->p_steps->plant_peak_mean_temp);
      Serial.println(left_leg->Curr_Combined);
      Serial.println(right_leg->Curr_Combined);
      Serial.println(left_leg->fsr_Combined_peak_ref);
      Serial.println(right_leg->fsr_Combined_peak_ref);
      Serial.println(left_leg->fsr_Toe_peak_ref);
      Serial.println(right_leg->fsr_Toe_peak_ref);
      Serial.println(left_leg->fsr_Heel_peak_ref);
      Serial.println(right_leg->fsr_Heel_peak_ref);
      Serial.println(left_leg->torque_calibration_value);
      Serial.println(right_leg->torque_calibration_value);


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
