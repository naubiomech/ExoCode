
/*
 * This function takes a torque input, computes the PID value (dependent on the control law), then turns the motors. 
 */

double pid(Leg* leg, double input) {
  if (CURRENT_CONTROL && MotorParams != 100) { //Simple Open-Loop Control
    if (leg->state == 3 && leg->PID_Setpoint != 0) {
      //leg->Vol = ((leg->PID_Setpoint/(TrqConstant * GearRatio * PulleyRatio * MotorEff * GearboxEff))/NomCurrent*2048) + leg->zero; //Setpoint/(Motor torque constant, gear reduction, pulley reduction, motor eff, gearbox eff)
      //leg->Vol = (0.37293*(leg->PID_Setpoint))/NomCurrent*2048.0; //Regression control, torque only
      leg->Vol = (0.275 * (leg->PID_Setpoint)) / NomCurrent * 2048.0;
      if (Control_Mode == 6 && leg->state == 3) {
        leg->Vol = -leg->Vol;
      }
    } else {
      leg->Vol = 0;
    }
  } else { //Closed-Loop Proportional Control
    if (Control_Mode == 6 && leg->state == 3) {
      leg->PID_Setpoint = -leg->PID_Setpoint;
    }
    leg->Input = input;
    //leg->pid.PID::Compute();
    leg->pid.Compute_KF(leg->KF);
    leg->Vol = leg->Output; //need to map
    //right_torque = right_leg->Output / 100;
    //left_torque = left_leg->Output / 100;
  }

  //leg->Vol = ((leg->PID_Setpoint/(TrqConstant * GearRatio * PulleyRatio * MotorEff * GearboxEff))/NomCurrent*2048); //For OL Data collection
  leg->Vol = leg->Vol + leg->zero; // Modify the span such that the PWM value is from 0 to 4096.0 instead of -2048.0 to 2048.0
  
  //analogWrite(leg->motor_ankle_pin, leg->Vol); //0 to 4096 writing for motor to get Input
  return leg->PID_Setpoint;
} 
