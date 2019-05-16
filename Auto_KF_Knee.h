// Updated 12/11/2018 to use mean torque instead of max torque


void Auto_KF_Knee(Leg* leg, int Control_Mode) {

  // take error during state 3
  if (leg->state == 3 && Control_Mode == 4) //If ID proportional
  {
    if (abs(leg->MaxPropSetpoint_Knee) != abs(leg->Max_Measured_Torque_Knee)) //If torque is under or over the setpoint
      leg->auto_KF_Knee_update = true;
  } // end if state=3
  //


  if (leg->state == 1 && leg->auto_KF_Knee_update) {
    //
    //    // Here we update the KF and then we disable the conditions that activate the if statement
    //

    if (Control_Mode == 4) { //If ID proportional

      if (leg->Max_Measured_Torque_Knee * leg->MaxPropSetpoint_Knee <= 0) {
        leg->auto_KF_Knee_update = false;
        return;
        //If signs are not concord no update of auto-KF
      }

      if (abs((leg->Max_Measured_Torque_Knee - (leg->MaxPropSetpoint_Knee)) / (leg->MaxPropSetpoint_Knee)) < 0.01) {
        leg -> auto_KF_Knee_update = false;
        //If error is less than 1 % no need to update KF
        return;
      }

      leg->KF_Knee = leg->KF_Knee - (leg->Max_Measured_Torque_Knee - (leg->MaxPropSetpoint_Knee)) / (leg->MaxPropSetpoint_Knee) * 0.6;
    }

    if isnan(leg->KF_Knee)
      leg->KF_Knee = 1;
 
    if (leg->KF_Knee >= leg->max_KF_Knee)
      leg->KF_Knee = leg->max_KF_Knee;
      else if (leg->KF_Knee <= leg->min_KF_Knee)
      leg->KF_Knee = leg->min_KF_Knee;

    leg->Max_Measured_Torque_Knee = 0;
    leg->MaxPropSetpoint_Knee = 0;
    leg->auto_KF_Knee_update = false; // to be able to do this cycle just once every step, i.e. during the whole state 1 I have to execute this cycle just once
  }

  return;

}
