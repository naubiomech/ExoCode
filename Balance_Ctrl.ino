//void Balance_Baseline() {
//
//  startTime = millis();
//
//  if (millis() - startTime < 1000)
//  {
//    for (int j = dim_FSR - 1; j >= 0; j--)                  //Sets up the loop to loop the number of spaces in the memory space minus 2, since we are moving all the elements except for 1
//    { // there are the number of spaces in the memory space minus 2 actions that need to be taken
//      *(left_leg->p_FSR_Array + j) = *(left_leg->p_FSR_Array + j - 1);                //Puts the element in the following memory space into the current memory space
//      *(right_leg->p_FSR_Array + j) = *(right_leg->p_FSR_Array + j - 1);
//
//      *(left_leg->p_FSR_Array_Heel + j) = *(left_leg->p_FSR_Array_Heel + j - 1);                //Puts the element in the following memory space into the current memory space
//      *(right_leg->p_FSR_Array_Heel + j) = *(right_leg->p_FSR_Array_Heel + j - 1);
//    }
//
//    //Get the FSR
//    *(left_leg->p_FSR_Array) = fsr(left_leg->fsr_sense_Toe);
//    *(right_leg->p_FSR_Array) = fsr(right_leg->fsr_sense_Toe);
//
//    *(left_leg->p_FSR_Array_Heel) = fsr(left_leg->fsr_sense_Heel);
//    *(right_leg->p_FSR_Array_Heel) = fsr(right_leg->fsr_sense_Heel);
//
//    left_leg->FSR_Toe_Balance_Baseline = 0;
//    left_leg->FSR_Heel_Balance_Baseline = 0;
//    right_leg->FSR_Toe_Balance_Baseline = 0;
//    right_leg->FSR_Heel_Balance_Baseline = 0;
//
//    for (int i = 0; i < dim_FSR; i++)
//    {
//      left_leg->FSR_Toe_Balance_Baseline +=  *(left_leg->p_FSR_Array + i);
//      right_leg->FSR_Toe_Balance_Baseline +=  *(right_leg->p_FSR_Array + i);
//
//      left_leg->FSR_Heel_Balance_Baseline +=  *(left_leg->p_FSR_Array_Heel + i);
//      right_leg->FSR_Heel_Balance_Baseline +=  *(right_leg->p_FSR_Array_Heel + i);
//    }
//
//    left_leg->FSR_Toe_Balance_Baseline /= dim_FSR;
//    left_leg->FSR_Heel_Balance_Baseline /= dim_FSR;
//    right_leg->FSR_Toe_Balance_Baseline /= dim_FSR;
//    right_leg->FSR_Heel_Balance_Baseline /= dim_FSR;
//
//  }
//
//}



void Balance_Baseline() {




  //  if (millis() - startTime < 5000)
  //  {
  //    left_leg->FSR_Toe_Balance_Baseline +=  fsr(left_leg->fsr_sense_Toe);
  //    right_leg->FSR_Toe_Balance_Baseline +=  fsr(right_leg->fsr_sense_Toe);
  //
  //    left_leg->FSR_Heel_Balance_Baseline +=  fsr(left_leg->fsr_sense_Heel);
  //    right_leg->FSR_Heel_Balance_Baseline += fsr(right_leg->fsr_sense_Heel);
  //
  //    count_balance++;
  //  } else {
  //
  //    left_leg->FSR_Toe_Balance_Baseline /= count_balance;
  //    left_leg->FSR_Heel_Balance_Baseline /= count_balance;
  //    right_leg->FSR_Toe_Balance_Baseline /= count_balance;
  //    right_leg->FSR_Heel_Balance_Baseline /= count_balance;
  //
  //    Serial.println(left_leg->FSR_Toe_Balance_Baseline);
  //    Serial.println(left_leg->FSR_Heel_Balance_Baseline);
  //    Serial.println(right_leg->FSR_Toe_Balance_Baseline);
  //    Serial.println(right_leg->FSR_Heel_Balance_Baseline);
  //    FLAG_BALANCE_BASELINE = 0;
  //
  //  }


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
    //
    //    count_balance++;
  } else {

//    left_leg->FSR_Toe_Balance_Baseline /= count_balance;
//    left_leg->FSR_Heel_Balance_Baseline /= count_balance;
//    right_leg->FSR_Toe_Balance_Baseline /= count_balance;
//    right_leg->FSR_Heel_Balance_Baseline /= count_balance;

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
  Serial.println(min(1,(leg->FSR_Toe_Average / leg->FSR_Toe_Balance_Baseline)) - min(1,(leg->FSR_Heel_Average / leg->FSR_Heel_Balance_Baseline)));

  return (min(1,(leg->FSR_Toe_Average / leg->FSR_Toe_Balance_Baseline)) - min(1,(leg->FSR_Heel_Average / leg->FSR_Heel_Balance_Baseline))) * (leg->Prop_Gain) * (leg->Setpoint_Ankle);

}

