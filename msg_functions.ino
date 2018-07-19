
void send_data_message_wc() //with COP
{
  bluetooth.print('S');
  bluetooth.print(',');
  bluetooth.print(R_sign*Average_Trq_RL);
  bluetooth.print(',');
  bluetooth.print(R_state);
  bluetooth.print(',');
  bluetooth.print(R_sign*PID_Setpoint_RL);
  bluetooth.print(',');
  //  bluetooth.print(fsr(fsr_sense_Right_Toe));
  //  bluetooth.print(fsr(fsr_sense_Right_Toe));
  bluetooth.print(Average_Volt_RL);
  bluetooth.print(',');
  bluetooth.print(Average_Volt_RL_Heel);
  bluetooth.print(',');
  bluetooth.print(Average_Trq_LL);
  bluetooth.print(',');
  bluetooth.print(L_state);
  bluetooth.print(',');
  bluetooth.print(L_sign*PID_Setpoint_LL);
  bluetooth.print(',');
  //  bluetooth.print(fsr(fsr_sense_Left_Toe));
  //  bluetooth.print(fsr(fsr_sense_Left_Toe));
  bluetooth.print(L_sign*Average_Volt_LL);
  bluetooth.print(',');
  bluetooth.print(Average_Volt_LL_Heel);
  bluetooth.print(',');
  bluetooth.print(analogRead(pin_err_LL));//bluetooth.print(flag_enable_catch_error);//SIG1
  bluetooth.print(',');
  bluetooth.print(analogRead(pin_err_RL));//bluetooth.print(motor_driver_count_err);//SIG2
  bluetooth.print(',');
  bluetooth.print(time_err_motor);//SIG3
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


