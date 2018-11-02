
void send_data_message_wc() //with COP
{
  bluetooth.print('S');
  bluetooth.print(',');

  // RIGHT
  bluetooth.print(right_leg->sign * right_leg->Average_Trq);
  bluetooth.print(',');
  bluetooth.print(right_leg->state);
  bluetooth.print(',');
  bluetooth.print(right_leg->sign * right_leg->PID_Setpoint);
  bluetooth.print(',');
  if (FLAG_TWO_TOE_SENSORS) {
    bluetooth.print(right_leg->fsr_percent_thresh_Toe * right_leg->fsr_Combined_peak_ref);
    bluetooth.print(',');
    if (FLAG_BALANCE) {
      bluetooth.print(right_leg->FSR_Toe_Average);
      bluetooth.print(',');
    } else {
      bluetooth.print(right_leg->FSR_Combined_Average);
      bluetooth.print(',');
    }
  } else {
    bluetooth.print(right_leg->fsr_percent_thresh_Toe * right_leg->fsr_Toe_peak_ref);
    bluetooth.print(',');
    bluetooth.print(right_leg->FSR_Toe_Average);
    bluetooth.print(',');
  }


  // LEFT
  bluetooth.print(left_leg->sign * left_leg->Average_Trq);
  bluetooth.print(',');
  bluetooth.print(left_leg->state);
  bluetooth.print(',');
  bluetooth.print(left_leg->sign * left_leg->PID_Setpoint);
  bluetooth.print(',');
  if (FLAG_TWO_TOE_SENSORS) {
    bluetooth.print(left_leg->fsr_percent_thresh_Toe * left_leg->fsr_Combined_peak_ref);
    bluetooth.print(',');
    if (FLAG_BALANCE) {
      bluetooth.print(left_leg->FSR_Toe_Average);
      bluetooth.print(',');
    } else {
      bluetooth.print(left_leg->FSR_Combined_Average);
      bluetooth.print(',');
    }
  } else {
    bluetooth.print(left_leg->fsr_percent_thresh_Toe * left_leg->fsr_Toe_peak_ref);
    bluetooth.print(',');
    bluetooth.print(left_leg->FSR_Toe_Average);
    bluetooth.print(',');
  }

  bluetooth.print((LED_BT_Voltage)); //SIG1
  
  bluetooth.print(',');
  bluetooth.print((right_leg->FSR_Heel_Average)); //SIG2
  bluetooth.print(',');

  bluetooth.print(left_leg->FSR_Toe_Average); //SIG3
  bluetooth.print(',');
  bluetooth.print(right_leg->FSR_Toe_Average); //SIG4

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
