// Updated 12/11/2018 to use mean torque instead of max torque


void Auto_KF(Leg* leg) {


  // take error during state 3
  if (leg->state == 3) {
    if (abs(leg->PID_Setpoint) >= 0.9 * abs(leg->Setpoint_Ankle * leg->coef_in_3_steps)) { //If target is greater than 90% of setpoint
      leg->Torque_Sum_90P += leg->Input;                // Sum torques
      leg->count_err ++;                                // Increment counter
      leg->auto_KF_update = true;                       // Raise auto KF flag
    } else {
      return;
    }
  }// end if state=3
  //


  if (leg->state == 1 && leg->auto_KF_update) {
    //
    //    // Here we update the KF and then we disable the conditions that activate the if statement
    //
    leg->Mean_Torque_90P = leg->Torque_Sum_90P / leg->count_err;


    if (leg->Mean_Torque_90P * leg->Setpoint_Ankle * leg->coef_in_3_steps <= 0) {
      // if the sign are not concord, no auto update of KF
      leg->auto_KF_update = false; //added after test of 11/7/18
      return;
    }

    if ( abs((leg->Mean_Torque_90P - (leg->Setpoint_Ankle * leg->coef_in_3_steps)) / (leg->Setpoint_Ankle * leg->coef_in_3_steps)) < 0.01) {
      leg->auto_KF_update = false; //added after test of 11/7/18
      return;
    }// if the error is less than 1% no need to change KF

    if (leg->coef_in_3_steps > 0) {
      leg->ERR = leg-> ERR + (leg->Mean_Torque_90P - (leg->Setpoint_Ankle * leg->coef_in_3_steps)) / (leg->Setpoint_Ankle * leg->coef_in_3_steps); //Calculate a running sum of the relative error
      leg->KF = leg->KF - ((leg->Mean_Torque_90P - (leg->Setpoint_Ankle * leg->coef_in_3_steps)) / (leg->Setpoint_Ankle * leg->coef_in_3_steps) * 0.6 + leg->ERR * 0.01); //changed from 0.4 to 0.6 after test of 11/7/18
    }

    if isnan(leg->KF) {
      leg->KF = 1;
    }

    if (leg->KF >= leg->max_KF) {
      leg->KF = leg->max_KF;
    }
    else if (leg->KF <= leg->min_KF) {
      leg->KF = leg->min_KF;
    }
    else {}

    leg->Mean_Torque_90P = 0;
    leg->Torque_Sum_90P = 0;
    leg->count_err = 0;
    leg->auto_KF_update = false; // to be able to do this cycle just once every step, i.e. during the whole state 1 I have to execute this cycle just once
  }// end of if state=1

  return;

}
