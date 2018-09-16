#include "Receive_and_Transmit.h"

// Peek is the variable used to identify the message received by matlab
// To understand the commands see the file .......... in the folder

void receive_and_transmit()
{

  int cmd_from_Gui = bluetooth.read();
  switch (cmd_from_Gui)
  {
  case COMM_CODE_REQUEST_DATA:
    send_data_message_wc();
    break;

  case COMM_CODE_GET_LEFT_ANKLE_SETPOINT:                                         //if MATLAB sent the character D
    *(data_to_send_point) = left_leg->Setpoint_Ankle;      //MATLAB is expecting to recieve the Torque Parameters
    send_command_message(COMM_CODE_GET_LEFT_ANKLE_SETPOINT, data_to_send_point, 1);
    Serial.print("Received Left Set ");
    Serial.println(left_leg->Setpoint_Ankle);
    Serial.println(left_leg->Dorsi_Setpoint_Ankle);
    break;

  case COMM_CODE_GET_RIGHT_ANKLE_SETPOINT:
    *(data_to_send_point) = right_leg->Setpoint_Ankle;
    send_command_message(COMM_CODE_GET_RIGHT_ANKLE_SETPOINT, data_to_send_point, 1);     //MATLAB is expecting to receive the Torque Parameters
    Serial.print("Received Right Set");
    Serial.println(right_leg->Setpoint_Ankle);
    Serial.println(right_leg->Dorsi_Setpoint_Ankle);
    break;

  case COMM_CODE_SET_LEFT_ANKLE_SETPOINT:
    receiveVals(16);                          //MATLAB is only sending 1 value, a double, which is 8 bytes
    left_leg->Previous_Setpoint_Ankle = left_leg->Setpoint_Ankle;
    left_leg->Previous_Dorsi_Setpoint_Ankle = left_leg->Dorsi_Setpoint_Ankle;
    memcpy(&left_leg->Setpoint_Ankle, &holdon, 8);                         //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
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
      Serial.println("Left Setpoint Negative, going to zero");
    } else {
      left_leg->Setpoint_Ankle = abs(left_leg->Setpoint_Ankle);                     //memory space pointed to by the variable Setpoint_Ankle.  Essentially a roundabout way to change a variable value, but since the bluetooth
      left_leg->Dorsi_Setpoint_Ankle = -abs(left_leg->Dorsi_Setpoint_Ankle);
      //Recieved the large data chunk chopped into bytes, a roundabout way was needed
      left_leg->p_steps->Setpoint = left_leg->sign * left_leg->Setpoint_Ankle;
      left_leg->Setpoint_Ankle_Pctrl = left_leg->Setpoint_Ankle;
      left_leg->activate_in_3_steps = 1;
      left_leg->num_3_steps = 0;
      left_leg->first_step = 1;
      left_leg->start_step = 0;
    }
    break;

  case COMM_CODE_SET_RIGHT_ANKLE_SETPOINT:
    receiveVals(16);                                         //MATLAB is only sending 1 value, a double, which is 8 bytes
    right_leg->Previous_Setpoint_Ankle = right_leg->Setpoint_Ankle;
    right_leg->Previous_Dorsi_Setpoint_Ankle = right_leg->Dorsi_Setpoint_Ankle;
    memcpy(&right_leg->Setpoint_Ankle, &holdon, 8);                         //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
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
      right_leg->p_steps->Setpoint = right_leg->sign * right_leg->Setpoint_Ankle;
      right_leg->activate_in_3_steps = 1;
      right_leg->num_3_steps = 0;
      right_leg->first_step = 1;
      right_leg->start_step = 0;
    }
    break;

  case COMM_CODE_START_TRIAL:
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);                                         //The GUI user is ready to start the trial so Motor is enabled
    stream = 1;                                                     //and the torque data is allowed to be streamed
    streamTimerCount = 0;
    timeElapsed = 0;
    break;

  case COMM_CODE_END_TRIAL:
    digitalWrite(MOTOR_ENABLE_PIN, LOW);                                         //The GUI user is ready to end the trial, so motor is disabled
    stream = 0;                                                    //and the torque data is no longer allowed to be streamed.
    break;

  case COMM_CODE_CALIBRATE_TORQUE:
    torque_calibration();
    write_torque_bias(left_leg->torque_address, left_leg->torque_calibration_value);
    write_torque_bias(right_leg->torque_address, right_leg->torque_calibration_value);
    break;

  case COMM_CODE_GET_LEFT_ANKLE_PID_PARAMS:
    *(data_to_send_point) = left_leg->kp_ankle;
    *(data_to_send_point + 1) = left_leg->kd_ankle;
    *(data_to_send_point + 2) = left_leg->ki_ankle;
    send_command_message(COMM_CODE_GET_LEFT_ANKLE_PID_PARAMS, data_to_send_point, 3);     //MATLAB is expecting to recieve the Torque Parameters
    break;

  case COMM_CODE_GET_RIGHT_ANKLE_PID_PARAMS:
    *(data_to_send_point) = right_leg->kp_ankle;
    *(data_to_send_point + 1) = right_leg->kd_ankle;
    *(data_to_send_point + 2) = right_leg->ki_ankle;
    send_command_message(COMM_CODE_GET_RIGHT_ANKLE_PID_PARAMS, data_to_send_point, 3);     //MATLAB is expecting to recieve the Torque Parameters
    break;

  case COMM_CODE_CALIBRATE_FSR:

    FSR_CAL_FLAG = 1;

    break;

  case COMM_CODE_SET_LEFT_PID_PARAMS:
    receiveVals(24);                                                //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
    //MATLAB Sent Kp, then Kd, then Ki.
    memcpy(&left_leg->kp_ankle, holdOnPoint, 8);                                  //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    memcpy(&left_leg->kd_ankle, holdOnPoint + 8, 8);                              //memory space pointed to by the variable kp_L.  Essentially a roundabout way to change a variable value, but since the bluetooth
    memcpy(&left_leg->ki_ankle, holdOnPoint + 16, 8);                             //Recieved the large data chunk chopped into bytes, a roundabout way was needed
    left_leg->ankle_pid.SetTunings(left_leg->kp_ankle, left_leg->ki_ankle, left_leg->kd_ankle);
    break;

  case COMM_CODE_SET_RIGHT_PID_PARAMS:
    receiveVals(24);                                                //MATLAB is sending 3 values, which are doubles, which have 8 bytes each
    //MATLAB Sent Kp, then Kd, then Ki.
    memcpy(&right_leg->kp_ankle, holdOnPoint, 8);                                  //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    memcpy(&right_leg->kd_ankle, holdOnPoint + 8, 8);                              //memory space pointed to by the variable kp_R.  Essentially a roundabout way to change a variable value, but since the bluetooth
    memcpy(&right_leg->ki_ankle, holdOnPoint + 16, 8);                             //Recieved the large data chunk chopped into bytes, a roundabout way was needed
    right_leg->ankle_pid.SetTunings(right_leg->kp_ankle, right_leg->ki_ankle, right_leg->kd_ankle);
    break;

  case COMM_CODE_CHECK_BLUETOOTH:
    *(data_to_send_point) = 0;
    *(data_to_send_point + 1) = 1;
    *(data_to_send_point + 2) = 2;
    send_command_message(COMM_CODE_CHECK_BLUETOOTH, data_to_send_point, 3);   //For the Arduino to prove to MATLAB that it is behaving, it will send back the character B
    break;

  case COMM_CODE_CHECK_MEMORY:
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
      if (FLAG_TWO_TOE_SENSORS) {
        left_leg->fsr_Combined_peak_ref = read_FSR_values(left_leg->address_FSR) + read_FSR_values(left_leg->address_FSR + sizeof(double) + 1);
        right_leg->fsr_Combined_peak_ref = read_FSR_values(right_leg->address_FSR) + read_FSR_values(right_leg->address_FSR + sizeof(double) + 1);
      } else {
        left_leg->fsr_Toe_peak_ref = read_FSR_values(left_leg->address_FSR);
        right_leg->fsr_Toe_peak_ref = read_FSR_values(right_leg->address_FSR);
        left_leg->fsr_Heel_peak_ref = read_FSR_values(left_leg->address_FSR + sizeof(double) + 1);
        right_leg->fsr_Heel_peak_ref = read_FSR_values(right_leg->address_FSR + sizeof(double) + 1);
      }
      *(data_to_send_point + 1) = 1;
      if (FLAG_TWO_TOE_SENSORS) {
        Serial.print("Left values Combined Toe and Heel: ");
        Serial.print(left_leg->fsr_Combined_peak_ref);
        Serial.print(", ");
        Serial.print("Right values: ");
        Serial.print(right_leg->fsr_Combined_peak_ref);
      } else {
        Serial.print("Left values Toe and Heel: ");
        Serial.print(left_leg->fsr_Toe_peak_ref);
        Serial.print(", ");
        Serial.print(left_leg->fsr_Heel_peak_ref);
        Serial.print(", ");
        Serial.print("Right values Toe and Hell: ");
        Serial.print(right_leg->fsr_Toe_peak_ref);
        Serial.print(", ");
        Serial.print(right_leg->fsr_Heel_peak_ref);
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

    send_command_message(COMM_CODE_CHECK_MEMORY, data_to_send_point, 3);
    left_leg->p_steps->voltage_peak_ref = left_leg->fsr_Combined_peak_ref;
    right_leg->p_steps->voltage_peak_ref = right_leg->fsr_Combined_peak_ref;

    left_leg->p_steps->plant_peak_mean = read_baseline(left_leg->baseline_address);
    right_leg->p_steps->plant_peak_mean = read_baseline(right_leg->baseline_address);
    left_leg->baseline_value = left_leg->p_steps->plant_peak_mean;
    right_leg->baseline_value = right_leg->p_steps->plant_peak_mean;
    Serial.print("Baseline ");
    Serial.print(left_leg->baseline_value);
    Serial.print(" , ");
    Serial.println(right_leg->baseline_value);
    break;

  case COMM_CODE_CLEAR_MEMORY:
    //------------------------------------------
    if (clean_torque_bias(left_leg->torque_address))
    {
      Serial.println("Clear Torque ");
    }
    else
    {
      Serial.println("No clear Torque");
    }
    if (clean_FSR_values(left_leg->address_FSR))
    {
      Serial.println("Clear FSR ");
    }
    else
    {
      Serial.println("No clear FSR");
    }
    //------------------------------------------
    if (clean_torque_bias(right_leg->torque_address))
    {
      Serial.println("Clear Torque ");
    }
    else
    {
      Serial.println("No clear Torque");
    }
    if (clean_FSR_values(right_leg->address_FSR))
    {
      Serial.println("Clear FSR ");
    }
    else
    {
      Serial.println("No clear FSR");
    }
    //------------------------------------------
    if (clean_FSR_values(left_leg->address_FSR + sizeof(double) + sizeof(char)))
    {
      Serial.println("Clear FSR ");
    }
    else
    {
      Serial.println("No clear FSR");
    }
    if (clean_FSR_values(right_leg->address_FSR + sizeof(double) + sizeof(char)))
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

  case COMM_CODE_SET_LEFT_ANKLE_KF_:
    receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&left_leg->KF, &holdon, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    break;

  case COMM_CODE_SET_RIGHT_ANKLE_KF:
    receiveVals(8);
    memcpy(&right_leg->KF, &holdon, 8);
    break;

  case COMM_CODE_GET_LEFT_ANKLE_KF:
    *(data_to_send_point) = left_leg->KF;
    send_command_message(COMM_CODE_GET_LEFT_ANKLE_KF, data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
    Serial.print("Checking left KF ");
    Serial.println(left_leg->KF);
    break;

  case COMM_CODE_GET_RIGHT_ANKLE_KF:
    *(data_to_send_point) = right_leg->KF;
    send_command_message(COMM_CODE_GET_RIGHT_ANKLE_KF, data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
    Serial.print("Checking right KF ");
    Serial.println(right_leg->KF);
    break;

  case COMM_CODE_SET_SMOOTHING_PARAMS:
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

    Serial.print("Set Smooth ");
    Serial.print(" ");
    Serial.print(N1);
    Serial.print(" ");
    Serial.print(N2);
    Serial.print(" ");
    Serial.print(N3);
    Serial.println();
    break;

  case COMM_CODE_GET_SMOOTHING_PARAMS:
    *(data_to_send_point) = N1;
    *(data_to_send_point + 1) = N2;
    *(data_to_send_point + 2) = N3;
    send_command_message(COMM_CODE_GET_SMOOTHING_PARAMS, data_to_send_point, 3);     //MATLAB is expecting to recieve the Torque Parameters
    Serial.print("Matlab Get Smooth ");
    Serial.print("");
    Serial.print(N1);
    Serial.print("");
    Serial.print(N2);
    Serial.print("");
    Serial.print(N3);
    Serial.println();

    break;

  case COMM_CODE_SET_LEFT_ANKLE_FREQ_BASELINE:
    left_leg->p_steps->flag_take_baseline = true;
    Serial.println("Left Freq Baseline ");
    break;

  case COMM_CODE_SET_RIGHT_ANKLE_FREQ_BASELINE:
    right_leg->p_steps->flag_take_baseline = true;
    Serial.println("Right Freq Baseline ");
    break;

  case COMM_CODE_ADJ_LEFT_ANKLE_N3:
    left_leg->p_steps->flag_N3_adjustment_time = true;
    Serial.println(" Left N3 Adj ");
    break;


  case COMM_CODE_ADJ_RIGHT_ANKLE_N3:
    right_leg->p_steps->flag_N3_adjustment_time = true;
    Serial.println(" Right N3 Adj ");
    break;

  case COMM_CODE_GET_LEFT_ANKLE_FSR_THRESHOLD:
    *(data_to_send_point) = left_leg->fsr_percent_thresh_Toe;
    send_command_message(COMM_CODE_GET_LEFT_ANKLE_FSR_THRESHOLD, data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
    Serial.print("Checking the left_leg->fsr_percent_thresh_Toe: ");
    Serial.println(left_leg->fsr_percent_thresh_Toe);
    break;

  case COMM_CODE_GET_RIGHT_ANKLE_FSR_THRESHOLD:
    *(data_to_send_point) = right_leg->fsr_percent_thresh_Toe;
    send_command_message(COMM_CODE_GET_RIGHT_ANKLE_FSR_THRESHOLD, data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
    Serial.print("Checking the right_leg->fsr_percent_thresh_Toe: ");
    Serial.println(right_leg->fsr_percent_thresh_Toe);
    break;

  case COMM_CODE_SET_LEFT_ANKLE_FSR_THRESHOLD:
    receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&left_leg->fsr_percent_thresh_Toe, &holdon, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    left_leg->p_steps->fsr_percent_thresh_Toe = left_leg->fsr_percent_thresh_Toe;
    Serial.print("Setting the left_leg->fsr_percent_thresh_Toe: ");
    Serial.println(left_leg->fsr_percent_thresh_Toe);
    break;

  case COMM_CODE_SET_RIGHT_ANKLE_FSR_THRESHOLD:
    receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&right_leg->fsr_percent_thresh_Toe, &holdon, 8);
    right_leg->p_steps->fsr_percent_thresh_Toe = right_leg->fsr_percent_thresh_Toe;
    Serial.print("Setting the fsr_percent_thresh_Rigth_Toe: ");
    Serial.println(right_leg->fsr_percent_thresh_Toe);
    break;

  case COMM_CODE_SET_LEFT_ANKLE_PERC:
    receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&(left_leg->p_steps->perc_l), &holdon, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    Serial.print("Setting the left_leg->p_steps->perc_l: ");
    Serial.println(left_leg->p_steps->perc_l);
    break;

  case COMM_CODE_SET_RIGHT_ANKLE_PERC:
    receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&(right_leg->p_steps->perc_l), &holdon, 8);                      //Copies 8 bytes (Just so happens to be the exact number of bytes MATLAB sent) of data from the first memory space of Holdon to the
    Serial.print("Setting the right_leg->p_steps->perc_l: ");
    Serial.println(right_leg->p_steps->perc_l);
    break;

  case COMM_CODE_CLEAN_BLUETOOTH_BUFFER:
    while (bluetooth.available() > 0) bluetooth.read();
    Serial.println("Buffer Clean");
    break;

  case COMM_CODE_STOP_LEFT_ANKLE_N3_ADJ:
    left_leg->p_steps->count_plant = 0;
    left_leg->p_steps->n_steps = 0;
    left_leg->p_steps->flag_start_plant = false;
    left_leg->p_steps->flag_take_average = false;
    left_leg->p_steps->flag_N3_adjustment_time = false;
    left_leg->p_steps->flag_take_baseline = false;
    left_leg->p_steps->torque_adj = false;
    left_leg->N3 = N3;
    Serial.print("Stop Left N3 adj, come back to: ");
    Serial.println(left_leg->N3);
    break;

  case COMM_CODE_STOP_RIGHT_ANKLE_N3_ADJ:
    right_leg->p_steps->count_plant = 0;
    right_leg->p_steps->n_steps = 0;
    right_leg->p_steps->flag_start_plant = false;
    right_leg->p_steps->flag_take_average = false;
    right_leg->p_steps->flag_N3_adjustment_time = false;
    right_leg->p_steps->flag_take_baseline = false;
    right_leg->p_steps->torque_adj = false;
    right_leg->N3 = N3;
    Serial.print("Stop Right N3 adj, come back to: ");
    Serial.println(right_leg->N3);
    break;

  case COMM_CODE_STOP_LEFT_ANKLE_TORQUE_ADJ:
    left_leg->Setpoint_Ankle = left_leg->p_steps->Setpoint;
    left_leg->p_steps->torque_adj = false;
    Serial.print("Stop Left TRQ adj, come back to: ");
    Serial.println(left_leg->Setpoint_Ankle);
    break;

  case COMM_CODE_STOP_RIGHT_ANKLE_TORQUE_ADJ:

    right_leg->Setpoint_Ankle = right_leg->p_steps->Setpoint;
    right_leg->p_steps->torque_adj = false;

    Serial.print("Stop Right TRQ adj, come back to: ");
    Serial.println(right_leg->Setpoint_Ankle);
    break;

  case COMM_CODE_SAVE_EXP_PARAMS:
    if (stream == 1) {
      Serial.print("Cannot save data during streaming ");
    } else {
      Serial.print("Saving Experimental Parameters ");

      if (FLAG_TWO_TOE_SENSORS) {
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
    }
    break;

  case COMM_CODE_MODIFY_LEFT_ANKLE_ZERO:
  {
    double app=0;
    receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&app, &holdon, 8);
    Serial.print("Modified the left zero of motor from : ");
    Serial.print(left_leg->zero);
    Serial.print(" to ");
    left_leg->zero = zero + app;
    Serial.println(left_leg->zero);
  }
  break;

  case COMM_CODE_MODIFY_RIGHT_ANKLE_ZERO:
  {
    double app=0;
    receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&app, &holdon, 8);
    Serial.print("Modified the right zero of motor from : ");
    Serial.print(right_leg->zero);
    Serial.print(" to ");
    right_leg->zero = zero + app;
    Serial.println(right_leg->zero);
  }
  break;

  case COMM_CODE_NEG_RIGHT_ANKLE_SIGN:
    right_leg->sign = -1;
    Serial.println(" Changed Sign in the Right torque ");
    break;

  case COMM_CODE_RESORE_RIGHT_ANKLE_SIGN:
    right_leg->sign = 1;
    Serial.println(" Restored the correct Sign the Right torque ");
    break;

  case COMM_CODE_SET_RIGHT_ANKLE_GAIN: // Receive Right Gain from GUI
    receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&right_leg->Prop_Gain, &holdon, 8);
    Serial.print(" Setting Right Gain for Proportional Ctrl: ");
    Serial.println(right_leg->Prop_Gain);
    break;

  case COMM_CODE_GET_RIGHT_ANKLE_GAIN: // Send Right Gain to GUI
    *(data_to_send_point) = right_leg->Prop_Gain;
    send_command_message(COMM_CODE_GET_RIGHT_ANKLE_GAIN, data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
    Serial.print(" Checking Right Gain for Proportional Ctrl: ");
    Serial.println(right_leg->Prop_Gain);
    break;

  case COMM_CODE_SET_LEFT_ANKLE_GAIN: // Receive Left Gain from GUI
    receiveVals(8);                                           //MATLAB is only sending 1 value, a double, which is 8 bytes
    memcpy(&left_leg->Prop_Gain, &holdon, 8);
    Serial.print(" Setting Left Gain for Proportional Ctrl: ");
    Serial.println(left_leg->Prop_Gain);
    break;

  case COMM_CODE_GET_LEFT_ANKLE_GAIN: // Send Left Gain to GUI
    *(data_to_send_point) = left_leg->Prop_Gain;
    send_command_message(COMM_CODE_GET_LEFT_ANKLE_GAIN, data_to_send_point, 1);     //MATLAB is expecting to recieve the Torque Parameters
    Serial.print(" Checking Left Gain for Proportional Ctrl: ");
    Serial.println(left_leg->Prop_Gain);
    break;

  case COMM_CODE_ACTIVATE_PROP_CTRL:
    if (not(FLAG_TWO_TOE_SENSORS)) {
      Old_Trq_time_volt = Trq_time_volt;
      Trq_time_volt = 2;
      right_leg->Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
      left_leg->Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
//        FLAG_TWO_TOE_SENSORS = true;
      FLAG_BALANCE = true;
      Serial.println(" Activate Balance Ctrl ");
    } else {
      Serial.println(" Cannoct Activate Balance Ctrl ");
    }

    break;

  case COMM_CODE_DEACTIVATE_PROP_CTRL:
    if (not(FLAG_TWO_TOE_SENSORS)) {
      Trq_time_volt = Old_Trq_time_volt;
      right_leg->p_steps->torque_adj = false;
      left_leg->p_steps->torque_adj = false;
      right_leg->Setpoint_Ankle = right_leg->p_steps->Setpoint;
      left_leg->Setpoint_Ankle = left_leg->p_steps->Setpoint;
      right_leg->Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
      left_leg->Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
//        FLAG_TWO_TOE_SENSORS = false;
      FLAG_BALANCE = false;

      Serial.println(" Deactivate Balance Ctrl ");
      Serial.println(Trq_time_volt);
    } else {
      Serial.println(" Cannoct Deactivate Balance Ctrl ");
    }
    break;


  case COMM_CODE_ACTIVATE_AUTO_KF:
    flag_auto_KF = 1;
    left_leg->KF = 1;
    right_leg->KF = 1;
    Serial.println(" Activate Auto KF ");
    break;

  case COMM_CODE_DEACTIVATE_AUTO_KF:
    flag_auto_KF = 0;
    Serial.println(" Deactivate Auto KF ");
    break;

  case COMM_CODE_ACTIVATE_PROP_PIVOT_CTRL:
    Old_Trq_time_volt = Trq_time_volt;
    Trq_time_volt = 3; // activate pivot proportional control
    right_leg->Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
    left_leg->Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
    Serial.println(" Activate Proportional Pivot Ctrl ");
    break;

  case COMM_CODE_DEACTIVATE_PROP_PIVOT_CTRL:
    Trq_time_volt = Old_Trq_time_volt;
    right_leg->p_steps->torque_adj = false;
    left_leg->p_steps->torque_adj = false;
    right_leg->Setpoint_Ankle = right_leg->p_steps->Setpoint;
    left_leg->Setpoint_Ankle = left_leg->p_steps->Setpoint;
    right_leg->Setpoint_Ankle_Pctrl = right_leg->p_steps->Setpoint;
    left_leg->Setpoint_Ankle_Pctrl = left_leg->p_steps->Setpoint;
    Serial.println(" Deactivate Proportional Pivot Ctrl ");
    break;

  case COMM_CODE_GET_BASELINE:
    // check baseline
    Serial.println("Check Baseline");
    Serial.println(left_leg->p_steps->plant_peak_mean);
    Serial.println(right_leg->p_steps->plant_peak_mean);
    left_leg->baseline_value = left_leg->p_steps->plant_peak_mean;
    right_leg->baseline_value = right_leg->p_steps->plant_peak_mean;
    *(data_to_send_point) = left_leg->p_steps->plant_peak_mean;
    *(data_to_send_point + 1) = right_leg->p_steps->plant_peak_mean;
    send_command_message(COMM_CODE_GET_BASELINE, data_to_send_point, 2);
    break;

  case COMM_CODE_CALC_BASELINE:
    // Calc baseline
    Serial.println(" Calc Baseline");
    left_leg->FSR_baseline_FLAG = 1;
    right_leg->FSR_baseline_FLAG = 1;
    base_1 = 0;
    base_2 = 0;
    left_leg->p_steps->count_plant_base = 0;
    right_leg->p_steps->count_plant_base = 0;
    right_leg->p_steps->flag_start_plant = false;
    left_leg->p_steps->flag_start_plant = false;
    right_leg->p_steps->Setpoint = 0;
    left_leg->p_steps->Setpoint = 0;
    break;
  }
  cmd_from_Gui = 0;
}





