
void send_data_message_wc() //with COP
{


  //Right Leg
  data_to_send[0] = (right_leg->sign * right_leg->Average_Trq);
  data_to_send[1] = right_leg->state;
  data_to_send[2] = (right_leg->sign * right_leg->PID_Setpoint);

  if (FLAG_TWO_TOE_SENSORS) {
    data_to_send[3] = (right_leg->fsr_percent_thresh_Toe * right_leg->fsr_Combined_peak_ref);
    data_to_send[4] = (right_leg->FSR_Combined_Average);
  } else if (FLAG_BALANCE) {
    data_to_send[3] = (right_leg->FSR_Toe_Average);
    data_to_send[4] = (right_leg->FSR_Heel_Average);
  } else {
    data_to_send[3] = (right_leg->fsr_percent_thresh_Toe * right_leg->fsr_Toe_peak_ref);
    data_to_send[4] = (right_leg->FSR_Toe_Average);
  }

  //Left Leg
  data_to_send[5] = (left_leg->sign * left_leg->Average_Trq);
  data_to_send[6] = left_leg->state;
  data_to_send[7] = (left_leg->sign * left_leg->PID_Setpoint);

  if (FLAG_TWO_TOE_SENSORS) {
    data_to_send[8] = (left_leg->fsr_percent_thresh_Toe * left_leg->fsr_Combined_peak_ref);
    data_to_send[9] = (left_leg->FSR_Combined_Average);
  } else if (FLAG_BALANCE) {
    data_to_send[8] = (left_leg->FSR_Toe_Average);
    data_to_send[9] = (left_leg->FSR_Heel_Average);
  } else {
    data_to_send[8] = (left_leg->fsr_percent_thresh_Toe * left_leg->fsr_Toe_peak_ref);
    data_to_send[9] = (left_leg->FSR_Toe_Average);
  }

  // Signals
  if (FLAG_BALANCE) {
    data_to_send[10] = (left_leg->COP_Foot_ratio);
    data_to_send[11] = (right_leg->COP_Foot_ratio);
  } else if (FLAG_BIOFEEDBACK) {
    data_to_send[10] = (pot(left_leg->Potentiometer_pin) + left_leg->Biofeedback_bias);
    data_to_send[11] = (pot(right_leg->Potentiometer_pin) + right_leg->Biofeedback_bias);
  }
  else {
    data_to_send[10] = (left_leg->motor_error);
    data_to_send[11] = (right_leg->motor_error);
  }
  if (FLAG_BIOFEEDBACK) {
    data_to_send[12] = (left_leg->Heel_Strike_baseline);
    data_to_send[13] = (int)100 * Freq;
  }
  else {
    data_to_send[12] = (left_leg->COP);
    data_to_send[13] = (right_leg->COP);
  }
  send_command_message('?', data_to_send, 14);
}


void send_command_message(char command_char, double* data_to_send, int number_to_send)
{
  bluetooth.write('S');
  bluetooth.write(command_char);
  bluetooth.write((char) number_to_send);
  float f = 0;
  char* trans = (char*) &f;
  for (int message_iterator = 0; message_iterator < number_to_send; message_iterator++)
  {
    f = data_to_send[message_iterator];
    for (int float_iter = 0; float_iter < sizeof(float); float_iter++) {
      bluetooth.write(trans[float_iter]);
    }
  }
}
