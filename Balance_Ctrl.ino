// the balance control return a torque level of assistance at the ankle depending on the distance between the current estimated COP and the reference (baseline) one.
// Control gain and parameters are decided in two different baseline processes. The first the steady estimate the COP at rest position. The second Dynamic, identifies the major variation of the COP
// during a dynamic task (swinging forward and backward). A gain allows to increase the torque provided at the ankle joint.
//--------------------------------------------------------------------

void Steady_Balance_Baseline() {


  if (millis() - startTime < 2000)
  {

    if ((millis() - startTime) / 100 >= 2 * count_steady_baseline) { // take two seconds to measure the baseline
      left_leg->FSR_Toe_Steady_Balance_Baseline +=  fsr(left_leg->fsr_sense_Toe);
      right_leg->FSR_Toe_Steady_Balance_Baseline +=  fsr(right_leg->fsr_sense_Toe);
      left_leg->FSR_Heel_Steady_Balance_Baseline +=  fsr(left_leg->fsr_sense_Heel);
      right_leg->FSR_Heel_Steady_Balance_Baseline += fsr(right_leg->fsr_sense_Heel);

      count_steady_baseline++;
    }
    
  } else {

    left_leg->FSR_Toe_Steady_Balance_Baseline /= count_steady_baseline;
    left_leg->FSR_Heel_Steady_Balance_Baseline /= count_steady_baseline;
    right_leg->FSR_Toe_Steady_Balance_Baseline /= count_steady_baseline;
    right_leg->FSR_Heel_Steady_Balance_Baseline /= count_steady_baseline;
    
//    Serial.println("Steady Balance Baseline");
//    Serial.println(left_leg->FSR_Toe_Steady_Balance_Baseline);
//    Serial.println(left_leg->FSR_Heel_Steady_Balance_Baseline);
//    Serial.println(right_leg->FSR_Toe_Steady_Balance_Baseline);
//    Serial.println(right_leg->FSR_Heel_Steady_Balance_Baseline);
    FLAG_STEADY_BALANCE_BASELINE = 0;

  }


}// end STEADY balance baseline



//--------------------------------------------------------------------

void Balance_Baseline() {


  if (millis() - startTime < 5000) //take 5 seconds to calculate the dynamic reference
  {
    if (left_leg->FSR_Toe_Balance_Baseline <=  fsr(left_leg->fsr_sense_Toe))
      left_leg->FSR_Toe_Balance_Baseline =  fsr(left_leg->fsr_sense_Toe);

    if (right_leg->FSR_Toe_Balance_Baseline <=  fsr(right_leg->fsr_sense_Toe))
      right_leg->FSR_Toe_Balance_Baseline =  fsr(right_leg->fsr_sense_Toe);


    if (left_leg->FSR_Heel_Balance_Baseline <= fsr(left_leg->fsr_sense_Heel))
      left_leg->FSR_Heel_Balance_Baseline =  fsr(left_leg->fsr_sense_Heel);

    if (right_leg->FSR_Heel_Balance_Baseline <= fsr(right_leg->fsr_sense_Heel))
      right_leg->FSR_Heel_Balance_Baseline = fsr(right_leg->fsr_sense_Heel);
    //
    //    count_balance++;
  } else {

    //    left_leg->FSR_Toe_Balance_Baseline /= count_balance;
    //    left_leg->FSR_Heel_Balance_Baseline /= count_balance;
    //    right_leg->FSR_Toe_Balance_Baseline /= count_balance;
    //    right_leg->FSR_Heel_Balance_Baseline /= count_balance;
//    Serial.println("Dynamic Balance Baseline");
//    Serial.println(left_leg->FSR_Toe_Balance_Baseline);
//    Serial.println(left_leg->FSR_Heel_Balance_Baseline);
//    Serial.println(right_leg->FSR_Toe_Balance_Baseline);
//    Serial.println(right_leg->FSR_Heel_Balance_Baseline);
    FLAG_BALANCE_BASELINE = 0;

  }


}// end balance baseline

//-----------------------------------------------------------
double Balance_Torque_ref(Leg * leg) {
  // first balance control which is linear to the current level of force
//  Serial.print("[ ");
//  Serial.print(leg->FSR_Toe_Average);
//  Serial.print(", ");
//  Serial.print(leg->FSR_Toe_Balance_Baseline);
//  Serial.print("] ");
//  Serial.print(" ");
//  Serial.print((leg->FSR_Toe_Average / leg->FSR_Toe_Balance_Baseline) - (leg->FSR_Heel_Average / leg->FSR_Heel_Balance_Baseline));
//  Serial.print(" -> ");
//  Serial.println(min(1, (leg->FSR_Toe_Average / leg->FSR_Toe_Balance_Baseline)) - min(1, (leg->FSR_Heel_Average / leg->FSR_Heel_Balance_Baseline)));

  return (min(1, (leg->FSR_Toe_Average / leg->FSR_Toe_Balance_Baseline)) - min(1, (leg->FSR_Heel_Average / leg->FSR_Heel_Balance_Baseline))) * (leg->Prop_Gain) * (leg->Setpoint_Ankle);

}


//-----------------------------------------

double Balance_Torque_ref_based_on_Steady(Leg * leg) {
  // balance control which depends on the steady and dynamic baseline.
  
  //  Serial.print("[ ");
  //  Serial.print(leg->FSR_Toe_Average);
  //  Serial.print(", ");
  //  Serial.print(leg->FSR_Toe_Balance_Baseline);
  //  Serial.print("] ");
  //  Serial.print(" ");
  //  Serial.print((leg->FSR_Toe_Average / leg->FSR_Toe_Balance_Baseline) - (leg->FSR_Heel_Average / leg->FSR_Heel_Balance_Baseline));
  //  Serial.print(" -> ");
  //  Serial.println(min(1, (leg->FSR_Toe_Average - leg->FSR_Toe_Steady_Balance_Baseline) / ( leg->FSR_Toe_Balance_Baseline - FSR_Toe_Steady_Balance_Baseline)) - min(1, (leg->FSR_Heel_Average / leg->FSR_Heel_Balance_Baseline)));

// Steady_multiplier Dynamic_multiplier to increase or decrease the sentitivitiness of the control to the baseline, it works as a gain that allows to engage the max torque earlier or later.
// in other words change the reference dimension of the estimated convex hull

    leg->COP_Toe_ratio = (leg->FSR_Toe_Average - leg->FSR_Toe_Steady_Balance_Baseline*leg->Steady_multiplier) / ( leg->FSR_Toe_Balance_Baseline*leg->Dynamic_multiplier - leg->FSR_Toe_Steady_Balance_Baseline*leg->Steady_multiplier);
    leg->COP_Heel_ratio = (leg->FSR_Heel_Average - leg->FSR_Heel_Steady_Balance_Baseline*leg->Steady_multiplier) / ( leg->FSR_Heel_Balance_Baseline*leg->Dynamic_multiplier - leg->FSR_Heel_Steady_Balance_Baseline*leg->Steady_multiplier);
  

    leg->COP_Foot_ratio = min(1, max(0, leg->COP_Toe_ratio)) - min(1, max(0, leg->COP_Heel_ratio));
  
  return (leg->COP_Foot_ratio) * (leg->Prop_Gain) * (leg->Setpoint_Ankle);

}

//-------------------------------------------
double Balance_Torque_COP_ref(Leg* leg) {
// balance control based onf the real COP
  if (leg->state == 3) {
    leg->COP = (leg->FSR_Toe_Average * leg->Toe_Pos + leg->FSR_Heel_Average * leg->Heel_Pos) / (leg->FSR_Heel_Average + leg->FSR_Toe_Average);
  }
  else {
    return 0;
  }

  if (leg->COP >= 0) {
    return (leg->COP / leg->Toe_Pos) * (leg->Prop_Gain) * (leg->Setpoint_Ankle);
  } else {
    return (-leg->COP / leg->Heel_Pos) * (leg->Prop_Gain) * (leg->Setpoint_Ankle); // Heel_Pos is negative
  }

}

