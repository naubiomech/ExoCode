
void Balance_Baseline() {


  if (millis() - startTime < 5000)
  {
    if (left_leg->FSR_Toe_Balance_Baseline <=  fsr(left_leg->fsr_sense_Toe))
      left_leg->FSR_Toe_Balance_Baseline =  fsr(left_leg->fsr_sense_Toe);

    if (right_leg->FSR_Toe_Balance_Baseline <=  fsr(right_leg->fsr_sense_Toe))
      right_leg->FSR_Toe_Balance_Baseline =  fsr(right_leg->fsr_sense_Toe);


    if (left_leg->FSR_Heel_Balance_Baseline <= fsr(left_leg->fsr_sense_Heel))
      left_leg->FSR_Heel_Balance_Baseline =  fsr(left_leg->fsr_sense_Heel);

    if (right_leg->FSR_Heel_Balance_Baseline <= fsr(right_leg->fsr_sense_Heel))
      right_leg->FSR_Heel_Balance_Baseline = fsr(right_leg->fsr_sense_Heel);

  } else {


    Serial.println(left_leg->FSR_Toe_Balance_Baseline);
    Serial.println(left_leg->FSR_Heel_Balance_Baseline);
    Serial.println(right_leg->FSR_Toe_Balance_Baseline);
    Serial.println(right_leg->FSR_Heel_Balance_Baseline);
    FLAG_BALANCE_BASELINE = 0;

  }


}


double Balance_Torque_ref(Leg * leg) {
  Serial.print("[ ");
  Serial.print(leg->FSR_Toe_Average);
  Serial.print(", ");
  Serial.print(leg->FSR_Toe_Balance_Baseline);
  Serial.print("] ");
  Serial.print(" ");
  Serial.print((leg->FSR_Toe_Average / leg->FSR_Toe_Balance_Baseline) - (leg->FSR_Heel_Average / leg->FSR_Heel_Balance_Baseline));
  Serial.print(" -> ");
  Serial.println(min(1, (leg->FSR_Toe_Average / leg->FSR_Toe_Balance_Baseline)) - min(1, (leg->FSR_Heel_Average / leg->FSR_Heel_Balance_Baseline)));

  return (min(1, (leg->FSR_Toe_Average / leg->FSR_Toe_Balance_Baseline)) - min(1, (leg->FSR_Heel_Average / leg->FSR_Heel_Balance_Baseline))) * (leg->Prop_Gain) * (leg->Setpoint_Ankle);

}

