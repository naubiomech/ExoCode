
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

  if (CURRENT_CONTROL && leg->PID_Setpoint!=0 && MotorParams!=100) {
    //leg->Vol = ((leg->PID_Setpoint/(TrqConstant * GearRatio * PulleyRatio * MotorEff * GearboxEff))/NomCurrent*2048) + leg->zero; //Setpoint/(Motor torque constant, gear reduction, pulley reduction, motor eff, gearbox eff)
    leg->Vol = (0.58314 + 0.33132*(leg->PID_Setpoint))/NomCurrent*2048 + leg->zero; //Regression control, torque only  
  } else if (CURRENT_DIAGNOSTICS && MotorParams!=100) {
    if (leg->Dorsi_Setpoint_Ankle==0) {
      leg->Vol = (leg->Setpoint_Ankle/NomCurrent*2048) + leg->zero;
    } else {
      leg->Vol = (leg->Dorsi_Setpoint_Ankle/NomCurrent*2048) + leg->zero;
    }
  } else if (MODEL_CONTROL && MotorParams!=100) {
    //if (leg->state == 3) {
      leg->Vol = (0.5668 + 0.29243*(leg->PID_Setpoint) + 0.0033*(leg->AverageSpeed) + 0.00077*(leg->PID_Setpoint * leg->AverageSpeed))/NomCurrent*2048 + leg->zero; //Regression control, averaged speed 
    //} else {
    //  leg->Vol = (-0.0549 + 0.2908*(leg->PID_Setpoint))/NomCurrent*2048 + leg->zero; //Regression control, torque only
    //}
  } else {
    leg->Vol = leg->Output + leg->zero; //need to map
  }

  if (PWM_CONTROL) {
     leg->Vol = leg->Vol*0.8 + 0.1*4096.0; 
  }

  analogWrite(leg->motor_ankle_pin, leg->Vol); //0 to 4096 writing for motor to get Input
}
