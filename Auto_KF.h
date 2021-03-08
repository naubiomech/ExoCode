// Updated 12/11/2018 to use mean torque instead of max torque


void Auto_KF(Leg* leg, int Control_Mode) {

  // take error during state 3
  if (leg->state == 3 && Control_Mode == 100) //If bang-bang
  {
    if (abs(leg->Setpoint_Ankle * leg->coef_in_3_steps) != abs(leg->Max_Measured_Torque)) { //If target is greater than 99% of setpoint
      leg->auto_KF_update = true;                       // Raise auto KF flag
    }
  } else if ((leg->state == 3 && Control_Mode == 3) || (leg->state == 3 && Control_Mode == 4) || (leg->state == 3 && Control_Mode == 6)) //If proportional
  {
    if (abs(leg->MaxPropSetpoint) != abs(leg->Max_Measured_Torque)) { //If torque is under or over the setpoint
      leg->auto_KF_update = true;
    }
  } // end if state=3
  //


  if (leg->state == 1 && leg->auto_KF_update) {
    //
    //    // Here we update the KF and then we disable the conditions that activate the if statement
    //

    if (Control_Mode == 100) { //If bang-bang

      if (leg->Max_Measured_Torque * leg->Setpoint_Ankle * leg->coef_in_3_steps <= 0) {
        // if the sign are not concord, no auto update of KF
        leg->auto_KF_update = false; //added after test of 11/7/18
        return;
      }

      if ( abs((leg->Max_Measured_Torque - (leg->Setpoint_Ankle * leg->coef_in_3_steps)) / (leg->Setpoint_Ankle * leg->coef_in_3_steps)) < 0.01) {
        leg->auto_KF_update = false; //added after test of 11/7/18
        return;
      }// if the error is less than 1% no need to change KF

      if (leg->coef_in_3_steps > 0) {
        leg->ERR = leg-> ERR + (leg->Max_Measured_Torque - (leg->Setpoint_Ankle * leg->coef_in_3_steps)) / (leg->Setpoint_Ankle * leg->coef_in_3_steps); //Calculate a running sum of the relative error
        leg->KF = leg->KF - ((leg->Max_Measured_Torque - (leg->Setpoint_Ankle * leg->coef_in_3_steps)) / (leg->Setpoint_Ankle * leg->coef_in_3_steps) * 0.6 + leg->ERR * 0.01); //changed from 0.4 to 0.6 after test of 11/7/18
      }
    } else if ((Control_Mode == 3) || (Control_Mode == 4) || (Control_Mode == 6)) { //If proportional

      if (leg->Max_Measured_Torque * leg->MaxPropSetpoint <= 0) {
        leg->auto_KF_update = false;
        return;
        //If signs are not concord no update of auto-KF
      }

      if (abs((leg->Max_Measured_Torque - (leg->MaxPropSetpoint)) / (leg->MaxPropSetpoint)) < 0.01) {
        leg -> auto_KF_update = false;
        //If error is less than 1 % no need to update KF
        return;
      }

      leg->KF = leg->KF - (leg->Max_Measured_Torque - (leg->MaxPropSetpoint)) / (leg->MaxPropSetpoint) * 0.6;

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
