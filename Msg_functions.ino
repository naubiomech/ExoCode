
void send_data_message_wc() //with COP
{
  bluetooth.print('S');
  bluetooth.print(',');

  // RIGHT
  bluetooth.print(R_sign * Average_Trq_RL);
  bluetooth.print(',');
  bluetooth.print(R_state);
  bluetooth.print(',');
  bluetooth.print(R_sign * PID_Setpoint_RL);
  bluetooth.print(',');
  //  bluetooth.print(fsr(fsr_sense_Right_Toe));
  //  bluetooth.print(fsr(fsr_sense_Right_Toe));
  //  bluetooth.print(Average_Volt_RL);
  bluetooth.print(fsr_percent_thresh_Right_Toe * fsr_Right_Combined_peak_ref);
  bluetooth.print(',');
  bluetooth.print(Combined_Average_RL);
  bluetooth.print(',');

  // LEFT
  bluetooth.print(L_sign * Average_Trq_LL);
  bluetooth.print(',');
  bluetooth.print(L_state);
  bluetooth.print(',');
  bluetooth.print(L_sign * PID_Setpoint_LL);
  bluetooth.print(',');
  //  bluetooth.print(fsr(fsr_sense_Left_Toe));
  //  bluetooth.print(fsr(fsr_sense_Left_Toe));
  //  bluetooth.print(Average_Volt_LL);
  bluetooth.print(fsr_percent_thresh_Left_Toe * fsr_Left_Combined_peak_ref);
  bluetooth.print(',');
  bluetooth.print(Combined_Average_LL);
  bluetooth.print(',');

  // OTHER
  //  bluetooth.print(timeElapsed); //SIG1
  //  bluetooth.print(',');
  //  bluetooth.print(L_COP_fun()); //SIG2
  //  bluetooth.print(',');
  //  bluetooth.print(R_COP_fun()); //SIG3
  //  //-------------------------------------
  //
  //
  //      bluetooth.print(FSR_Average_LL_Heel); //SIG1
  //      bluetooth.print(',');
  //      bluetooth.print(FSR_Average_RL_Heel); //SIG2-
  //      bluetooth.print(',');
  //      bluetooth.print(min(10, fabs(L_p_steps->curr_voltage / L_p_steps->plant_peak_mean))); //SIG3
  //      bluetooth.print(',');
  //      bluetooth.print(min(10, fabs(R_p_steps->curr_voltage / R_p_steps->plant_peak_mean))); //SIG4
  //
  //
  //  //-----------------------------

  //-------------------------------------


//  bluetooth.print(*p_L_Max_FSR_Ratio); //SIG1
//  bluetooth.print(',');
//  bluetooth.print(*p_R_Max_FSR_Ratio); //SIG2


  bluetooth.print(Time_error_counter_LL); //SIG1
  bluetooth.print(',');
  bluetooth.print(Time_error_counter_RL); //SIG2
  bluetooth.print(',');
  bluetooth.print(*p_L_FSR_Ratio); //SIG3
  bluetooth.print(',');
  bluetooth.print(*p_R_FSR_Ratio); //SIG4


  //-----------------------------

  //  bluetooth.print((analogRead(pin_err_LL) <= 5) || (analogRead(pin_err_RL) <= 5)); //SIG1
  //  bluetooth.print(',');
  //  bluetooth.print(analogRead(pin_err_LL)); //SIG2
  //  bluetooth.print(',');
  //  bluetooth.print(analogRead(pin_err_RL)); //SIG3
  //  bluetooth.print(',');
  //  bluetooth.print(analogRead(pin_err_RL)); //SIG4



  ////  bluetooth.print(analogRead(A6)); //SIG4

  //  bluetooth.print(analogRead(fsr_sense_Left_Heel)+analogRead(fsr_sense_Left_Toe)); //SIG2
  //  bluetooth.print(',');
  //  bluetooth.print(analogRead(fsr_sense_Right_Heel)+analogRead(fsr_sense_Right_Toe)); //SIG3

  bluetooth.print(',');
  bluetooth.println('Z');
}

void send_command_message(char command_char, double* data_point, int number_to_send)
{
  bluetooth.print('S');
  bluetooth.print(command_char);
  bluetooth.print(',');
  for (int message_iterator = 0; message_iterator < number_to_send; message_iterator++)
  {
    bluetooth.print(*(data_point + message_iterator));
    bluetooth.print(',');
  }
  bluetooth.println('Z');
}

//---------------------------------------------------------------------
// Receive msg from matlab
/*
  int receive_msg(msg* p_msg_l) {

  int k = 0;

  while ((k < 75))
  {
    while (bluetooth.available() > 0)
    {
      int fromBluetooth = bluetooth.read();
      p_msg_l->msg_array[k] = fromBluetooth;               //Store all recieved bytes in subsequent memory spaces
      k = k + 1;                                  //Increments memory space
    }
  }

  p_msg_l->special_character = p_msg_l->msg_array[1];

  if ((p_msg_l->msg_array[0] == 'S') && (p_msg_l->msg_array[74] == 'E')) {
    Serial.println("Received correct msg");
    char arr[8];

    for (int j = 0; j < 8; j++) {

      for (int i = 0; i < 8 ; i++) {
        arr[i] = p_msg_l->msg_array[j * (8 + 1) + 2 + i];
      }

      memcpy(p_msg_l->N_array[j], arr, 8);
    }

  } else {
    Serial.println("Not Received correct msg");
    return 0;
  }

  return 1;
  }
*/


