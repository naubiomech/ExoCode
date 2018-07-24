#include "Board.h"

void pid(double input, int Left_or_Right)
{
  if (Left_or_Right == 1)
  {
    if ((abs(input) > 25))
    {


      KF_RL = 0;
      double old_L_state_L = L_state;
      L_state = 9;
      send_data_message_wc();

      digitalWrite(onoff, LOW);
      stream = 0;
      digitalWrite(LED_PIN, LOW);
      L_state = old_L_state_L;

    }
    Input_LL = input;
    PID_LL.Compute_KF(KF_LL);
    Vol_LL = Output_LL + zero; //need to map
    analogWrite(MOTOR_LEFT_ANKLE_PIN, Vol_LL); //0 to 4096 writing for motor to get Input
  }
  if (Left_or_Right == 2)
  {
    if ((abs(input) > 25))
    {
      KF_RL = 0;
      double old_R_state_R = R_state;
      R_state = 9;
      send_data_message_wc();
      digitalWrite(onoff, LOW);
      stream = 0;
      digitalWrite(LED_PIN, LOW);
      R_state = old_R_state_R;
    }
    Input_RL = input;
    PID_RL.Compute_KF(KF_RL);

    Vol_RL = Output_RL + zero; //need to map
    analogWrite(MOTOR_RIGHT_ANKLE_PIN, Vol_RL); //0 to 4096 writing for motor to get Input
  }
}
