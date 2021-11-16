
/*
 * This function takes a torque input, computes the PID value (dependent on the control law), then turns the motors. 
 */

double pid(Leg* leg, double input) {
  if (CURRENT_CONTROL && leg->PID_Setpoint != 0 && MotorParams != 100 && leg->state == 3) { //Simple Open-Loop Control
    leg->Vol = (0.275 * (leg->PID_Setpoint)) / NomCurrent * 2048.0;
    if (Control_Mode == 6 && leg->state == 3) {
      leg->Vol = -leg->Vol;
    }
  } else if (CURRENT_DIAGNOSTICS && MotorParams != 100) { //Diagnostics Mode
    if (leg->Dorsi_Setpoint_Ankle == 0) {
      leg->Vol = leg->Setpoint_Ankle / NomCurrent * 2048.0;
    } else {
      leg->Vol = leg->Dorsi_Setpoint_Ankle / NomCurrent * 2048.0;
    }
  } else if (MODEL_CONTROL && MotorParams != 100) {
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
  return leg->Vol;
} 
