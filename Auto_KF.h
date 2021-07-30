// Updated 12/11/2018 to use mean torque instead of max torque


void Auto_KF(Leg* leg, int Control_Mode) {

  // take error during state 3
  if ((leg->state == 3 && Control_Mode == 3) || (leg->state == 3 && Control_Mode == 4) || (leg->state == 3 && Control_Mode == 6)) //If proportional
  {
    if (abs(leg->MaxPropSetpoint) != abs(leg->Max_Measured_Torque)) { //If torque is under or over the setpoint
      leg->auto_KF_update = true;
    }
  } // end if state=3
  //

  if (leg->state == 1 && leg->auto_KF_update) {
    //
    //    // Here we update the KF and then we disable the conditions that activate the if statement

    if ((Control_Mode == 3) || (Control_Mode == 4) || (Control_Mode == 6)) { //If proportional

      if (leg->Max_Measured_Torque * leg->MaxPropSetpoint <= 0) {
        leg->auto_KF_update = false;
        return;
        //If signs are not concord no update of auto-KF
      }

      if (abs((leg->Max_Measured_Torque - (leg->MaxPropSetpoint)) / (leg->MaxPropSetpoint)) < 0.01) {
        leg->auto_KF_update = false;
        //If error is less than 1 % no need to update KF
        return;
      }
      leg->KF = leg->KF - (leg->Max_Measured_Torque - (leg->MaxPropSetpoint)) / (leg->MaxPropSetpoint)*0.6;

    } else {}

    if (isnan(leg->KF)) {
      leg->KF = 1;
    }

    if (leg->KF >= leg->max_KF) {
      leg->KF = leg->max_KF;
    }
    else if (leg->KF <= leg->min_KF) {
      leg->KF = leg->min_KF;
    }
    else {}
    leg->Max_Measured_Torque = 0;
    leg->MaxPropSetpoint = 0;
    leg->auto_KF_update = false; // to be able to do this cycle just once every step, i.e. during the whole state 1 I have to execute this cycle just once
  }

  return;

}
