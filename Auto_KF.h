//int flag_auto_KF = 0;
//
//void Auto_KF() {
//
//  // take error in state 3
//  if (left_leg->state == 3) {
//    //    left_leg->Input is the average of the measured torque
//    //    left_leg->PID_Stepoint is the reference
//    Serial.print(" Left Error ");
//    Serial.println(left_leg->PID_Setpoint - left_leg->Input );
//    left_leg->ERR += (left_leg->PID_Setpoint - left_leg->Input );
//    left_leg->count_err++;
//  }
//
//  if (left_leg->state == 1) {
//
//    left_leg->ERR = left_leg->ERR / left_leg->count_err;
//    if ((left_leg->count_err != 0)) {
//      Serial.print("Left ERR ");
//      Serial.println(left_leg->ERR);
//    }
//    else {
//
//    }
//    left_leg->count_err = 0;
//
//
//
//    if (left_leg->ERR > max_ERR) {
//      left_leg->KF += 0.05;
//    }
//    else if (left_leg->ERR < min_ERR) {
//      left_leg->KF -= 0.05;
//    }
//    else {}
//
//    if (left_leg->KF >= left_leg->max_KF)
//      left_leg->KF = left_leg->max_KF;
//    else if (left_leg->KF <= left_leg->min_KF)
//      left_leg->KF = left_leg->min_KF;
//    else {}
//
//    Serial.print("New left_leg->KF ");
//    Serial.println(left_leg->KF);
//    left_leg->ERR = 0;
//  }
//
//
//  if (right_leg->state == 3) {
//    //    right_leg->Input is the average of the measured torque
//    //    right_leg->PID_Stepoint is the reference
//    Serial.print(" Right Error ");
//    Serial.println(right_leg->PID_Setpoint - right_leg->Input );
//    right_leg->ERR += (right_leg->PID_Setpoint - right_leg->Input );
//    right_leg->count_err++;
//  }
//  if (right_leg->state == 1) {
//
//    right_leg->ERR = -right_leg->ERR / right_leg->count_err; // because the right has a different sign
//    if ((right_leg->count_err != 0)) {
//      Serial.print(" Right ERR ");
//      Serial.println(right_leg->ERR);
//    }
//    else {
//
//    }
//    right_leg->count_err = 0;
//
//
//    if (right_leg->ERR > max_ERR) {
//      right_leg->KF += 0.05;
//    }
//    else if (right_leg->ERR < min_ERR) {
//      right_leg->KF -= 0.05;
//    }
//    else {}
//
//    if (right_leg->KF >= right_leg->max_KF)
//      right_leg->KF = right_leg->max_KF;
//    else if (right_leg->KF <= right_leg->min_KF)
//      right_leg->KF = right_leg->min_KF;
//    else {}
//
//    Serial.print("New right_leg->KF ");
//    Serial.println(right_leg->KF);
//    right_leg->ERR = 0;
//  }
//
//
//  //  left_leg->ERR += ()
//  // adjust KF
//
//  return;
//}





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


