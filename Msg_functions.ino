
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
  bluetooth.print(fsr_percent_thresh_Left_Toe * fsr_Left_Combined_peak_ref);
  bluetooth.print(',');
  bluetooth.print(Combined_Average_LL);
  bluetooth.print(',');


  bluetooth.print(FSR_Average_LL_Heel); //SIG1
  bluetooth.print(',');
  bluetooth.print(FSR_Average_RL_Heel); //SIG2-
  bluetooth.print(',');
  bluetooth.print(min(10, fabs(L_p_steps->curr_voltage / L_p_steps->plant_peak_mean))); //SIG3
  bluetooth.print(',');
  bluetooth.print(min(10, fabs(R_p_steps->curr_voltage / R_p_steps->plant_peak_mean))); //SIG4

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
