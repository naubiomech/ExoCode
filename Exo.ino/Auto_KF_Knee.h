// Updated 12/11/2018 to use mean torque instead of max torque


void Auto_KF_Knee(Leg* leg, int Control_Mode) {

  // take error during state 3  // TN 8/30/19
  if (leg->state == 3 && Control_Mode == 100) //If bang-bang
  {
    if (abs(leg->Setpoint_Knee * leg->coef_in_3_steps_Knee) != abs(leg->Max_Measured_Torque_Knee)) { //If target is greater than 99% of setpoint
      leg->auto_KF_Knee_update = true;                       // Raise auto KF flag
    }
  } else if (leg->state == 3 && Control_Mode == 4) //If ID proportional
  {
    if (abs(leg->MaxPropSetpoint_Knee) != abs(leg->Max_Measured_Torque_Knee)) //If torque is under or over the setpoint
      leg->auto_KF_Knee_update = true;
  } // end if state=3
  //

  // TN 8/30/19
  if (leg->state == 1 && leg->auto_KF_Knee_update) {
    //
    //    // Here we update the KF and then we disable the conditions that activate the if statement
    if (Control_Mode == 100) { //If bang-bang
      if (leg->Max_Measured_Torque_Knee * leg->Setpoint_Knee * leg->coef_in_3_steps_Knee <= 0) {
        // if the sign are not concord, no auto update of KF
        leg->auto_KF_Knee_update = false; //added after test of 11/7/18
        return;
      }

      if ( abs((leg->Max_Measured_Torque_Knee - (leg->Setpoint_Knee * leg->coef_in_3_steps_Knee)) / (leg->Setpoint_Knee * leg->coef_in_3_steps_Knee)) < 0.01) {
        leg->auto_KF_Knee_update = false; //added after test of 11/7/18
        return;
      }// if the error is less than 1% no need to change KF

      if (leg->coef_in_3_steps_Knee > 0) {
        leg->ERR_Knee = leg-> ERR_Knee + (leg->Max_Measured_Torque_Knee - (leg->Setpoint_Knee * leg->coef_in_3_steps_Knee)) / (leg->Setpoint_Knee * leg->coef_in_3_steps_Knee); //Calculate a running sum of the relative error
        leg->KF_Knee = leg->KF_Knee - ((leg->Max_Measured_Torque_Knee - (leg->Setpoint_Knee * leg->coef_in_3_steps_Knee)) / (leg->Setpoint_Knee * leg->coef_in_3_steps_Knee) * 0.6 + leg->ERR_Knee * 0.01); //changed from 0.4 to 0.6 after test of 11/7/18
      }
    } else if (Control_Mode == 4) { //If ID proportional

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
    } else {}

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
