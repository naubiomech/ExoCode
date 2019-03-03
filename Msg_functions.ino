
void send_data_message_wc() //with COP
{

//Right Leg
      Sig_1=(int) 100* (right_leg->sign * right_leg->Average_Trq);
      Sig_2=right_leg->state;
      Sig_3=(int) 100* (right_leg->sign * right_leg->PID_Setpoint);
      
      if (FLAG_TWO_TOE_SENSORS) {
      Sig_4=(int) 100*(right_leg->fsr_percent_thresh_Toe * right_leg->fsr_Combined_peak_ref);
      Sig_5=(int) 100*(right_leg->FSR_Combined_Average);
        }    
      else
      {Sig_4=(int) 100*(right_leg->fsr_percent_thresh_Toe * right_leg->fsr_Toe_peak_ref);
       Sig_5=(int) 100*(right_leg->FSR_Toe_Average);
      }

//Left Leg
      Sig_6=(int) 100* (left_leg->sign * left_leg->Average_Trq);
      Sig_7=left_leg->state;
      Sig_8=(int) 100* (left_leg->sign * left_leg->PID_Setpoint);
      
      if (FLAG_TWO_TOE_SENSORS) {
      Sig_9=(int) 100*(left_leg->fsr_percent_thresh_Toe * left_leg->fsr_Combined_peak_ref);
      Sig_10=(int) 100*(left_leg->FSR_Combined_Average);
        } 
      else
      {Sig_9=(int) 100*(left_leg->fsr_percent_thresh_Toe * left_leg->fsr_Toe_peak_ref);
       Sig_10=(int) 100*(left_leg->FSR_Toe_Average);
      }

// Signals
    if (FLAG_BALANCE) {
        Signal_1=(int) 100*(left_leg->COP_Foot_ratio);
        Signal_2=(int) 100*(right_leg->COP_Foot_ratio);
      } else if (FLAG_BIOFEEDBACK) {
        Signal_1=(int) 100*(pot(left_leg->Potentiometer_pin) + left_leg->Biofeedback_bias);
        Signal_2=(int) 100*(pot(right_leg->Potentiometer_pin) + right_leg->Biofeedback_bias);
      }
      else {
        Signal_1=(int) 100*(left_leg->motor_error);
        Signal_2=(int) 100*(right_leg->motor_error);
        //Signal_1=(int) 100*(left_leg->Time_error_counter);
        //Signal_2=(int) 100*(right_leg->Time_error_counter);
        
      }
      if (FLAG_BIOFEEDBACK) {
        Signal_3=(int) 100*(left_leg->Heel_Strike_baseline);
        Signal_4= (int)100* Freq;
      }
      else {
        Signal_3=(int) 100*(left_leg->COP);
        Signal_4=(int) 100*(right_leg->COP);
      }

  // Send Messages
  bluetooth.print('S');
  bluetooth.print(',');
  bluetooth.print(Sig_1);
  bluetooth.print(',');
  bluetooth.print(Sig_2); 
  bluetooth.print(',');
  bluetooth.print(Sig_3);
  bluetooth.print(',');
  bluetooth.print(Sig_4);
  bluetooth.print(',');
  bluetooth.print(Sig_5);
  bluetooth.print(',');
  bluetooth.print(Sig_6);
  bluetooth.print(',');
  bluetooth.print(Sig_7);
  bluetooth.print(',');
  bluetooth.print(Sig_8);
  bluetooth.print(',');
  bluetooth.print(Sig_9);
  bluetooth.print(',');
  bluetooth.print(Sig_10);
  bluetooth.print(',');
  bluetooth.print(Signal_1);
  bluetooth.print(',');
  bluetooth.print(Signal_2);
  bluetooth.print(',');
  bluetooth.print(Signal_3);
  bluetooth.print(',');
  bluetooth.print(Signal_4);
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
