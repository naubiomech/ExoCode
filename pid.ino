#include "Board.h"

void pid(double input, int Left_or_Right)
{
  if (Left_or_Right == 1)
  {
    if ((abs(input) > 25))
    {


      right_leg->KF = 0;
      double old_L_state_L = left_leg->state;
      left_leg->state = 9;
      send_data_message_wc();

      digitalWrite(onoff, LOW);
      stream = 0;
      digitalWrite(LED_PIN, LOW);
      left_leg->state = old_L_state_L;

    }
    left_leg->Input = input;
    left_leg->pid.Compute_KF(left_leg->KF);
    left_leg->Vol = left_leg->Output + zero; //need to map
    analogWrite(MOTOR_LEFT_ANKLE_PIN, left_leg->Vol); //0 to 4096 writing for motor to get Input
  }
  if (Left_or_Right == 2)
  {
    if ((abs(input) > 25))
    {
      right_leg->KF = 0;
      double old_R_state_R = right_leg->state;
      right_leg->state = 9;
      send_data_message_wc();
      digitalWrite(onoff, LOW);
      stream = 0;
      digitalWrite(LED_PIN, LOW);
      right_leg->state = old_R_state_R;
    }
    right_leg->Input = input;
    right_leg->pid.Compute_KF(right_leg->KF);

    right_leg->Vol = right_leg->Output + zero; //need to map
    analogWrite(MOTOR_RIGHT_ANKLE_PIN, right_leg->Vol); //0 to 4096 writing for motor to get Input
  }
}
