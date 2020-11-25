
void send_data_message_wc() //with COP
{
  
  //Executes if running iOS app
  if (iOS_Flag)
  {
  //Right Leg
  data_to_send[0] = (right_leg->sign * right_leg->Average_Trq);
  data_to_send[1] = right_leg->state;
  data_to_send[2] = (right_leg->sign * right_leg->PID_Setpoint);
    
  //Left Leg
  data_to_send[3] = (left_leg->sign * left_leg->Average_Trq);
  data_to_send[4] = left_leg->state;
  data_to_send[5] = (left_leg->sign * left_leg->PID_Setpoint);
  
  send_command_message('?', data_to_send, 6);
  }

  //Executes if running MATLAB
  else if (!iOS_Flag)
  {
  //Right Leg
  data_to_send[0] = (right_leg->sign * right_leg->Average_Trq);//motor_ankle_speed(right_leg->motor_speed_pin)/123/(60/25);
  //data_to_send[0] = right_leg->Average_Trq*69.559*4*0.36/0.22; //Futek load cell
  data_to_send[1] = left_leg->state_swing_duration; //right_leg->state;//right_leg->AnkleAverageAngle;
  data_to_send[2] = left_leg->state_swing_start_time;//(right_leg->sign * right_leg->PID_Setpoint);//right_leg->AnkleAverageSpeed/360*60;

  if (FLAG_ONE_TOE_SENSOR) {
    data_to_send[3] = right_leg->Average_Trq; //(right_leg->fsr_percent_thresh_Toe * right_leg->fsr_Combined_peak_ref);
    data_to_send[4] = (right_leg->FSR_Combined_Average);//current(right_leg->motor_current_pin);
  } else if (FLAG_BALANCE) {
    //data_to_send[3] = 12;//right_leg->FSR_Toe_Average);
    //data_to_send[4] = 13;//(right_leg->FSR_Heel_Average);
  } else if (FLAG_BIOFEEDBACK) { //YF
    //data_to_send[3] = right_leg->score;
    //data_to_send[4] = left_leg->score;
  } else {
    data_to_send[3] = (right_leg->fsr_percent_thresh_Toe * right_leg->fsr_Toe_peak_ref);
    data_to_send[4] = (right_leg->FSR_Toe_Average);
//    data_to_send[3] = (right_leg->fsr_percent_thresh_Toe * right_leg->fsr_Combined_peak_ref);
//    data_to_send[4] = (right_leg->FSR_Combined_Average);
  }

  //Left Leg
  data_to_send[5] =  (left_leg->sign * left_leg->Average_Trq);//-motor_ankle_speed(left_leg->motor_speed_pin)/123/(60/25);
  //data_to_send[5] = left_leg->Average_Trq*100.000; //Transducer raw voltage output
  data_to_send[6] = left_leg->state;//ankle_angle(left_leg);
  data_to_send[7] = (left_leg->sign * left_leg->PID_Setpoint);//left_leg->AnkleAverageSpeed/360*60;

  if (FLAG_ONE_TOE_SENSOR) {
    data_to_send[8] =  (left_leg->fsr_percent_thresh_Toe * left_leg->fsr_Combined_peak_ref); //left_leg->Average_Trq;
    data_to_send[9] = (left_leg->FSR_Combined_Average);// current(left_leg->motor_current_pin);
  } else if (FLAG_BALANCE) {
    //data_to_send[8] = (left_leg->FSR_Toe_Average);
    //data_to_send[9] = (left_leg->FSR_Heel_Average);
  } else {
    data_to_send[8] = (left_leg->FSR_Heel_Average);//(left_leg->fsr_percent_thresh_Toe * left_leg->fsr_Toe_peak_ref);
    data_to_send[9] = (left_leg->FSR_Toe_Average);
    //data_to_send[8] = (left_leg->fsr_percent_thresh_Toe * right_leg->fsr_Combined_peak_ref);
    //data_to_send[9] = (left_leg->FSR_Combined_Average);
  }

  // Signals
  if (FLAG_BALANCE) {
    data_to_send[10] = 100*3.3*(analogRead(left_leg->ankle_angle_pin)-2048)/2048; //right_leg->Vol;//(left_leg->COP_Foot_ratio);
    data_to_send[11] = 100*3.3*(analogRead(right_leg->ankle_angle_pin)-2048)/2048;//current(right_leg->motor_current_pin);//(right_leg->COP_Foot_ratio);
  } else if (FLAG_BIOFEEDBACK) {
    data_to_send[10] = 3;//right_leg->stridelength_update;
    data_to_send[11] = 4;//left_leg->stridelength_update;
  }
  else {
//    data_to_send[10] = (left_leg->TM_data);
//    data_to_send[11] = (right_leg->TM_data);
    //data_to_send[10] = right_leg->Vol; //(left_leg->COP_Foot_ratio);
    //data_to_send[11] = 100*3.3*(analogRead(right_leg->ankle_angle_pin)-2048)/2048; //(right_leg->COP_Foot_ratio);
    data_to_send[10] = 100*3.3*(analogRead(left_leg->ankle_angle_pin)-2048)/2048;//current(right_leg->motor_current_pin);
    data_to_send[11] = right_leg->sign * motor_ankle_speed(right_leg->motor_speed_pin);
  }
  if (FLAG_BIOFEEDBACK) {
    data_to_send[12] = right_leg->stridelength_target;
    data_to_send[13] = left_leg->stridelength_target;
  }
  else {
    //data_to_send[12] =  left_leg->Vol;//Numerical derivative of angle
    //data_to_send[13] = ((right_leg->Vol - 0.1*4096)/0.8 - right_leg->zero) / 2048 * NomCurrent;
    //data_to_send[13] = right_leg->AnkleAverageAngle; //Ankle angle
    //data_to_send[13] = 100*3.3*(analogRead(left_leg->ankle_angle_pin)-2048)/2048;
    data_to_send[12] = current(left_leg->motor_current_pin);
    data_to_send[13] = left_leg->sign * motor_ankle_speed(left_leg->motor_speed_pin);
    //data_to_send[12] = left_leg->trig_number;//SS  6/23/2020
    //data_to_send[13] = left_leg->Trigger;//SS  6/23/2020
  }
  send_command_message('?', data_to_send, 14);
  }
  
}
  


void send_command_message(char command_char, double* data_to_send, int number_to_send)
{
  
  if (iOS_Flag)
  {
    bluetooth.write('S');
    bluetooth.write((char) command_char);
    bluetooth.print((int) number_to_send);
    float f = 0;
    for (int message_iterator = 0; message_iterator < number_to_send; message_iterator++)
    {
      f = data_to_send[message_iterator];
      double buffer_for_int = f * 100.0;
      int int_to_send = (int) buffer_for_int;
      bluetooth.print(int_to_send);
      bluetooth.write('n');
    }
  }
  
  else if (!iOS_Flag)
  {
    bluetooth.write('S');
    bluetooth.write(command_char);
    bluetooth.write((char) number_to_send);
    float f = 0;
    char* trans = (char*) &f;
    for (int message_iterator = 0; message_iterator < number_to_send; message_iterator++)
    {
      f = data_to_send[message_iterator];
      for (int float_iter = 0; float_iter < sizeof(float); float_iter++) 
      {
      bluetooth.write(trans[float_iter]);
      }
  }
  }
 
}
