
// In the pid function there's a check on torque reading. If the torque measured is >25 for 10 times it means we have a problem with the cable and we stop the system.
// Changing the number 10 we increase or decrease the sensitivity of the system to false positives at the same time we introduce a delay in the stopping action.
// if the torque measured <25 the counter is reset.

void pid(Leg* leg, double input){
  if ((abs(input) > 35)) //Was 25, increased to accomodate large exo
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
