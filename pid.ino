
// In the pid function there's a check on torque reading. If the torque measured is >25 for 10 times it means we have a problem with the cable and we stop the system.
// Changing the number 10 we increase or decrease the sensitivity of the system to false positives at the same time we introduce a delay in the stopping action.
// if the torque measured <25 the counter is reset.

void pid(Leg* leg, double input) {
  if (!CURRENT_CONTROL) {
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
  
    } else {}
  } else {
    
  }

  if (CURRENT_CONTROL && leg->PID_Setpoint!=0 && MotorParams!=100 && leg->state==3) { //Simple Open-Loop Control
    //leg->Vol = ((leg->PID_Setpoint/(TrqConstant * GearRatio * PulleyRatio * MotorEff * GearboxEff))/NomCurrent*2048) + leg->zero; //Setpoint/(Motor torque constant, gear reduction, pulley reduction, motor eff, gearbox eff)
    //leg->Vol = (0.37293*(leg->PID_Setpoint))/NomCurrent*2048.0; //Regression control, torque only  
    leg->Vol = (0.275*(leg->PID_Setpoint))/NomCurrent*2048.0;
    if (Control_Mode == 6 && leg->state == 3) {
      leg->Vol = -leg->Vol;
    }
  } else if (CURRENT_DIAGNOSTICS && MotorParams!=100) { //Diagnostics Mode
    if (leg->Dorsi_Setpoint_Ankle==0) {
      leg->Vol = leg->Setpoint_Ankle/NomCurrent*2048.0; 
    } else {
      leg->Vol = leg->Dorsi_Setpoint_Ankle/NomCurrent*2048.0;
    }
  } else if (MODEL_CONTROL && MotorParams!=100) {
    leg->Vol = (0.34284*(leg->PID_Setpoint) + 0.023564*(leg->AverageSpeed) + 0.0043038*(leg->PID_Setpoint * leg->AverageSpeed))/NomCurrent*2048; //Regression control, complex model
    if (Control_Mode == 6 && leg->state == 3) {
      leg->Vol = -leg->Vol;
    }
  } else { //Closed-Loop Proportional Control
    if (Control_Mode == 6 && leg->state == 3) {
      leg->PID_Setpoint = -leg->PID_Setpoint;
    }
    leg->Input = input;
    leg->pid.Compute_KF(leg->KF);
    leg->Vol = leg->Output; //need to map
  }

  leg->Vol = leg->Vol + leg->zero; // Modify the span such that the PWM value is from 0 to 4096.0 instead of -2048.0 to 2048.0

  if (PWM_CONTROL) {
     leg->Vol = leg->Vol*0.8 + 0.1*4096.0;  // Motor drivers need the PWM to be between 10% and 90%
  }
  analogWrite(leg->motor_ankle_pin, leg->Vol); //0 to 4096 writing for motor to get Input
}
