#include "Board.h"

void pid(Leg* leg, double input) {
  if ((abs(input) > 25))
  {
    leg->torque_error_counter++;
    if (leg->torque_error_counter >= 10) {
      leg->KF = 0;
      double old_L_state_L = leg->state;
      leg->state = 9;
      send_data_message_wc();

      digitalWrite(onoff, LOW);
      stream = 0;
      digitalWrite(LED_PIN, LOW);
      leg->state = old_L_state_L;
      leg->torque_error_counter = 0;
    }

  } else {
    leg->torque_error_counter = 0;
  }

  leg->Input = input;
  leg->pid.Compute_KF(leg->KF);
  //  Serial.print(" ZERO: ");
  //  Serial.print(leg->zero);
  leg->Vol = leg->Output + leg->zero; //need to map
  analogWrite(leg->motor_ankle_pin, leg->Vol); //0 to 4096 writing for motor to get Input
}
