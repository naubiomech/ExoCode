
// In the pid function there's a check on torque reading. If the torque measured is >25 for 10 times it means we have a problem with the cable and we stop the system.
// Changing the number 10 we increase or decrease the sensitivity of the system to false positives at the same time we introduce a delay in the stopping action.
// if the torque measured <25 the counter is reset.

void pid(Leg* leg, double input) {
  if ((abs(input) > 35)) //Was 25, increased to accomodate large exo
  {
    leg->torque_error_counter++;
    if (leg->torque_error_counter >= 10) {
      //leg->KF = 0;
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
  }
  //leg->Input = input;
  //leg->pid.Compute_KF(leg->KF);
  //  Serial.print(" ZERO: ");
  //  Serial.print(leg->zero);
  //leg->Vol = leg->Output + leg->zero; //need to map

  leg->Vol = (leg->PID_Setpoint/((13.6/1000.0) * 51.0 * (70.4/10.3) * 0.89 * 0.70))/7.58*2048;
//  Serial.print("Desired Current: ");
//  Serial.println(leg->Vol);


  if (PWM_CONTROL) {
    leg->Vol = leg->Vol + 2048.0; //Map to 0 to 4096
//    Serial.print("Desired Current Bits: ");
//    Serial.println(leg->Vol);
  }


  analogWrite(leg->motor_ankle_pin, leg->Vol); //0 to 4096 writing for motor to get Input
}
