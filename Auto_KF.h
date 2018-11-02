int flag_auto_KF = 0;

void Auto_KF(Leg* leg) {

  // take error in state 3
  if (leg->state == 3) {

    if (abs(leg->Input) > abs(leg->ERR)) {
      leg->ERR = leg->Input; // put max Input
      leg->auto_KF_update = true;
    }
  }

  if (leg->state == 1 && leg->auto_KF_update) {
    Serial.print(" Ref ");
    Serial.print(leg->Setpoint_Ankle * leg->coef_in_3_steps);
    Serial.print(" , Max Torque measured ");
    Serial.print(leg->ERR);
    Serial.println(" ");


    leg->KF = 1 + (1 - leg->ERR / (leg->Setpoint_Ankle * leg->coef_in_3_steps));
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
    leg->auto_KF_update = false;
  }

  return;

}


