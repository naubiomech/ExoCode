
void Auto_KF(Leg* leg) {


  // take error during state 3
  if (leg->state == 3) {
    //    left_leg->Input is the average of the measured torque
    //    left_leg->PID_Stepoint is the reference

    if (abs(leg->Input) > abs(leg->Max_Measured_Torque)) {
      leg->Max_Measured_Torque = leg->Input; // put max Input
      leg->auto_KF_update = true;
    }
  }// end if state=3

  if (leg->state == 1 && leg->auto_KF_update) {

    // Here we update the KF and then we disable the conditions that activate the if statement

    Serial.print(" Ref ");
    Serial.print(leg->Setpoint_Ankle * leg->coef_in_3_steps);
    Serial.print(" , Max Torque measured ");
    Serial.print(leg->Max_Measured_Torque);
    //    Serial.print(" , Coeff: ");
    //    Serial.print(leg->Max_Measured_Torque / (leg->Setpoint_Ankle * leg->coef_in_3_steps));
    Serial.println(" ");


    if (leg->Max_Measured_Torque * leg->Setpoint_Ankle * leg->coef_in_3_steps <= 0) {
      // if the sign are not concord, no auto update of KF
      leg->auto_KF_update = false; //added after test of 11/7/18
      return;
    }

    //    if (leg->Setpoint_Ankle * leg->coef_in_3_steps > 0) {

    if ( abs((leg->Max_Measured_Torque - (leg->Setpoint_Ankle * leg->coef_in_3_steps)) / (leg->Setpoint_Ankle * leg->coef_in_3_steps)) < 0.05) {
      leg->auto_KF_update = false; //added after test of 11/7/18
      return;
    }// if the error is less than the 5% no need 

    leg->KF = leg->KF - (leg->Max_Measured_Torque - (leg->Setpoint_Ankle * leg->coef_in_3_steps)) / (leg->Setpoint_Ankle * leg->coef_in_3_steps) * 0.6; //changed from 0.4 to 0.6 after test of 11/7/18

    //    }
    //    if (leg->Setpoint_Ankle * leg->coef_in_3_steps < 0) {
    //      leg->KF = leg->KF + (leg->Max_Measured_Torque - (leg->Setpoint_Ankle * leg->coef_in_3_steps)) / (leg->Setpoint_Ankle * leg->coef_in_3_steps) * 0.4;
    //    }

    // Now we have to be prepared for the next step and hence we have to reset the max measured torque variable
    leg->Max_Measured_Torque = 0;

    Serial.print("Desired leg->KF ");
    Serial.println(leg->KF);

    if (leg->KF >= leg->max_KF) {
      leg->KF = leg->max_KF;
    }
    else if (leg->KF <= leg->min_KF) {
      leg->KF = leg->min_KF;
    }
    else {}

    Serial.print("Actual leg->KF ");
    Serial.println(leg->KF);
    leg->auto_KF_update = false; // to be able to do this cycle just once every step, i.e. during the whole state 1 I have to execute this cycle just once
  }// end of if state=1

  return;

}


