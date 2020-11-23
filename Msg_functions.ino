
void send_data_message_wc() //with COP
{

  //Right Leg
  data_to_send[0] = left_leg->Angle_Thigh;//(right_leg->sign * right_leg->Average_Trq);
  data_to_send[1] = left_leg->AnkleMoment;//right_leg->state;
  data_to_send[2] = left_leg->Angle_Shank;///(right_leg->sign * right_leg->PID_Setpoint);

  if (FLAG_TOE_HEEL_SENSORS) { // TN 5/8/19
    data_to_send[3] = left_leg->Angle_Foot;//((right_leg->fsr_percent_thresh_Toe + right_leg->fsr_percent_thresh_Heel) * right_leg->fsr_Combined_peak_ref);
    data_to_send[4] = left_leg->AngularAccAve_Thigh;///(right_leg->FSR_Combined_Average);
  } else if (FLAG_BALANCE) {
    data_to_send[3] = (right_leg->FSR_Toe_Average);
    data_to_send[4] = (right_leg->FSR_Heel_Average);
  } else if (FLAG_BIOFEEDBACK) { //YF
    data_to_send[3] = right_leg->score;
    data_to_send[4] = left_leg->score;
  } else {
    data_to_send[3] = left_leg->Angle_Foot;//((right_leg->fsr_percent_thresh_Toe + right_leg->fsr_percent_thresh_Heel) * right_leg->fsr_Combined_peak_ref);
    data_to_send[4] = left_leg->AngularAccAve_Thigh;//(right_leg->FSR_Combined_Average);
  }

  //Left Leg
  data_to_send[5] = left_leg->AngularAccAve_Shank;//(left_leg->sign * left_leg->Average_Trq);
  data_to_send[6] = left_leg->AngularVelAve_Foot;//left_leg->AnkleMoment_Mod;//left_leg->state;
  data_to_send[7] = left_leg->AngularAccAve_Foot;//(left_leg->sign * left_leg->PID_Setpoint);

  if (FLAG_TOE_HEEL_SENSORS) {  // TN 5/8/19
    data_to_send[8] = (left_leg->FSR_Toe_Abs);//((left_leg->fsr_percent_thresh_Toe + left_leg->fsr_percent_thresh_Heel) * left_leg->fsr_Combined_peak_ref);
    data_to_send[9] = (left_leg->FSR_Heel_Abs);//(left_leg->FSR_Combined_Average);
  } else if (FLAG_BALANCE) {
    data_to_send[8] = (left_leg->FSR_Toe_Average);
    data_to_send[9] = (left_leg->FSR_Heel_Average);
  } else {
    data_to_send[8] = (left_leg->FSR_Toe_Abs);//((left_leg->fsr_percent_thresh_Toe + left_leg->fsr_percent_thresh_Heel) * left_leg->fsr_Combined_peak_ref);
    data_to_send[9] = (left_leg->FSR_Heel_Abs);//(left_leg->FSR_Combined_Average);
  }

  // Signals
  if (FLAG_BALANCE) {
    data_to_send[10] = (left_leg->COP_Foot_ratio);
    data_to_send[11] = (right_leg->COP_Foot_ratio);
  } else if (FLAG_BIOFEEDBACK) {
    data_to_send[10] = right_leg->stridelength_update;
    data_to_send[11] = left_leg->stridelength_update;
  }
  else if (Flag_Knee_Cfg == true) {  // TN 5/17/19
    data_to_send[10] = right_leg->sign * right_leg->PID_Setpoint_Knee;
    data_to_send[11] = left_leg->sign * left_leg->PID_Setpoint_Knee;  // TN 5/17/19

    //data_to_send[10] = (right_leg->PID_Setpoint_Knee);   // TN 5/13/19
    // data_to_send[11] = (left_leg->PID_Setpoint_Knee);   // TN 5/13/19
  }
  else  {
    data_to_send[10] = left_leg->KneeMoment;//(right_leg->sign * right_leg->PID_Setpoint_Knee);   // TN 5/13/19
    data_to_send[11] = left_leg->AngularVelAve_Shank;//left_leg->KneeMoment_Mod;//left_leg->Angle_Thigh;//(left_leg->sign * left_leg->PID_Setpoint_Knee);   // TN 5/13/19
  }

if (FLAG_BIOFEEDBACK) {
    data_to_send[12] = right_leg->stridelength_target;
    data_to_send[13] = left_leg->stridelength_target;
  }
  else {
    data_to_send[12] = left_leg->HipMoment; //right_leg->Angular_Impulse_Knee;//(right_leg->sign * right_leg->Average_Trq_Knee); // SS 9/17/2019
    data_to_send[13] = left_leg->AngularVelAve_Thigh;//left_leg->HipMoment_Mod; //(left_leg->sign * left_leg->Average_Trq_Knee);  // SS 9/17/2019
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
