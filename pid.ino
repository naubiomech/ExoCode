
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
  leg->Input = input;
  leg->pid.Compute_KF(leg->KF);
//    Serial.print(" ZERO: ");
//    Serial.print(leg->zero);

  if (CURRENT_CONTROL && leg->PID_Setpoint!=0 && MotorParams!=100) {
    leg->Vol = ((leg->PID_Setpoint/(TrqConstant * GearRatio * PulleyRatio * MotorEff * GearboxEff))/NomCurrent*2048) + leg->zero; //Setpoint/(Motor torque constant, gear reduction, pulley reduction, motor eff, gearbox eff)
  } else if (CURRENT_DIAGNOSTICS && MotorParams!=100) {
    //leg->Vol = (leg->Setpoint_Ankle/NomCurrent*2048) + leg->zero;
    //leg->Vol = (-0.26985 + 0.24933*(leg->PID_Setpoint) + 0.05811*(ankle_speed(leg->ankle_speed_pin) - 0.00079793*(leg->PID_Setpoint * ankle_speed(leg->ankle_speed_pin))/NomCurrent*2048 + leg->zero;
    leg->Vol = (-0.25593 + 0.24983*(leg->PID_Setpoint) + 0.048642*(ankle_speed(leg->motor_speed_pin)) - 0.0015455*(leg->PID_Setpoint * ankle_speed(leg->motor_speed_pin)))/NomCurrent*2048 + leg->zero;
  } else if (MODEL_CONTROL && MotorParams!=100) {
    //leg->Vol = (0.95795 - 0.22166*(leg->PID_Setpoint) - 0.47695*(leg->state) + 
    // 0.026898*(leg->AverageSpeed) + 0.17103*(leg->PID_Setpoint*leg->state) + 0.002358*(leg->PID_Setpoint * leg->AverageSpeed) + 
    // 0.011848*(leg->state * leg->AverageSpeed))/NomCurrent*2048 + leg->zero;
    leg->Vol = (1.0827 - 0.22015*(leg->PID_Setpoint) - 0.54852*(leg->state) + 
     0.021852*(ankle_speed(leg->motor_speed_pin)) + 0.16851*(leg->PID_Setpoint*leg->state) + 0.001462*(leg->PID_Setpoint * ankle_speed(leg->motor_speed_pin)) + 
     0.0090642*(leg->state * ankle_speed(leg->motor_speed_pin)))/NomCurrent*2048 + leg->zero;
  } else {
    leg->Vol = leg->Output + leg->zero; //need to map
  }

  if (PWM_CONTROL) {
     leg->Vol = leg->Vol*0.8 + 0.1*4096.0; 
  }

  analogWrite(leg->motor_ankle_pin, leg->Vol); //0 to 4096 writing for motor to get Input
}
