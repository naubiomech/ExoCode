int flag_auto_KF = 0;

double ERR_LL, ERR_RL;
double max_KF_LL = 1.8;
double max_KF_RL = 1.8;
double min_KF_LL = 0.8;
double min_KF_RL = 0.8;
int count_err_LL, count_err_RL;

double max_ERR = 0.20;
double min_ERR = -0.20;

void Auto_KF() {

  // take error in state 3
  if (L_state == 3) {
    //    Input_LL is the average of the measured torque
    //    PID_Stepoint_LL is the reference
    Serial.print(" Left Error ");
    Serial.println(PID_Setpoint_LL - Input_LL );
    ERR_LL += (PID_Setpoint_LL - Input_LL );
    count_err_LL++;
  }

  if (L_state == 1) {

    ERR_LL = ERR_LL / count_err_LL;
    if ((count_err_LL != 0)) {
      Serial.print("Left ERR ");
      Serial.println(ERR_LL);
    }
    else {

    }
    count_err_LL = 0;



    if (ERR_LL > max_ERR) {
      KF_LL += 0.05;
    }
    else if (ERR_LL < min_ERR) {
      KF_LL -= 0.05;
    }
    else {}

    if (KF_LL >= max_KF_LL)
      KF_LL = max_KF_LL;
    else if (KF_LL <= min_KF_LL)
      KF_LL = min_KF_LL;
    else {}

    Serial.print("New KF_LL ");
    Serial.println(KF_LL);
    ERR_LL = 0;
  }


  if (R_state == 3) {
    //    Input_RL is the average of the measured torque
    //    PID_Stepoint_RL is the reference
    Serial.print(" Right Error ");
    Serial.println(PID_Setpoint_RL - Input_RL );
    ERR_RL += (PID_Setpoint_RL - Input_RL );
    count_err_RL++;
  }
   if (R_state == 1) {

    ERR_RL = -ERR_RL / count_err_RL; // because the right has a different sign
    if ((count_err_RL != 0)) {
      Serial.print(" Right ERR ");
      Serial.println(ERR_RL);
    }
    else {

    }
    count_err_RL = 0;


    if (ERR_RL > max_ERR) {
      KF_RL += 0.05;
    }
    else if (ERR_RL < min_ERR) {
      KF_RL -= 0.05;
    }
    else {}

    if (KF_RL >= max_KF_RL)
      KF_RL = max_KF_RL;
    else if (KF_RL <= min_KF_RL)
      KF_RL = min_KF_RL;
    else {}

    Serial.print("New KF_RL ");
    Serial.println(KF_RL);
    ERR_RL = 0;
  }

  
  //  ERR_LL += ()
  // adjust KF

  return;
}
