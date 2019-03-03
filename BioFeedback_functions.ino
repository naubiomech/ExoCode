void takeHeelStrikeAngle(Leg* leg) {

  leg->Heel_Strike += (pot(leg->Potentiometer_pin) + leg->Biofeedback_bias);                //Take baseline knee angle data
  leg->Heel_Strike_Count++;
  Serial.print("Heel strike sum and n of steps: ");
  Serial.print(leg->Heel_Strike);
  Serial.print(" , ");
  Serial.println(leg->Heel_Strike_Count);

  if (leg->BioFeedback_Baseline_flag == true) {
    Heel_strike_update_and_biofeedback(leg);
  }
  return;
}




//void biofeedback()
//{
//  Frequency = map(pot(pot_cal_RL), pot_A_min, pot_A_max, 200, 1000);    //Scale pot value to 200-1000
//  //
//  //  analogWrite(jack, 5);
//  //  tone(jack, 500, 100);
//  //  delay(Frequency);                           //delay changes with pot value (knee angle)
//  //  analogWrite(jack, 5);
//  //  tone(jack, 500, 100);
//  //  delay(Frequency);
//}




void BioFeedback_Baseline(Leg* leg) {

  if (leg->BioFeedback_Baseline_flag == false) {
    if (leg->Heel_Strike_Count >= leg->n_step_biofeedback_base)
    {
      Serial.print("UPDATING BASELINE for the BioFeedback : ");
      Serial.println(leg->Heel_Strike);
      Serial.println(leg->Heel_Strike_Count);
      leg->Heel_Strike_baseline = leg->Heel_Strike / leg->Heel_Strike_Count;
      leg->BioFeedback_Baseline_flag = true;
      leg->BIO_BASELINE_FLAG = false;

      Serial.println(leg->Heel_Strike_baseline);
      leg->Heel_Strike_Count = 0;
      leg->Heel_Strike = 0;
    }
  }
  return;
}


void Heel_strike_update_and_biofeedback(Leg * leg) {

  Serial.println("Inside Heel strike");
  if (leg->Heel_Strike_Count >= leg->n_step_biofeedback)
  {
    Serial.println("Inside first if Heel strike");
    leg->Heel_Strike_mean = leg->Heel_Strike / leg->Heel_Strike_Count;

    Serial.print("BioFeedback_Baseline = ");
    Serial.print(leg->Heel_Strike_baseline);
    Serial.print(" ; Difference between mean and desired : ");
    Serial.println(abs(leg->Heel_Strike_mean - leg->BioFeedback_desired));


    if (abs(leg->Heel_Strike_mean - leg->BioFeedback_desired) < leg->min_knee_error && leg->BioFeedback_Baseline_flag == true) {
      leg->NO_Biofeedback = true;
      Serial.println("NO Biofeedback true");
    } else {
      leg->NO_Biofeedback = false;
      Serial.println("NO Biofeedback false");
    }

    if (abs(leg->Heel_Strike_mean - leg->BioFeedback_desired) >= leg->min_knee_error && leg->BioFeedback_Baseline_flag == true)
    {
      Serial.println("Inside second if Heel strike");
      leg->Biofeedback_ctrl_max = min(abs((20) / (leg->BioFeedback_desired - leg->Heel_Strike_baseline)), 3);
      leg->Frequency = -min(1, abs((leg->BioFeedback_desired - leg->Heel_Strike_mean) / (leg->BioFeedback_desired - leg->Heel_Strike_baseline)) / leg->Biofeedback_ctrl_max) * (BioFeedback_Freq_max - BioFeedback_Freq_min) + BioFeedback_Freq_max;
      Serial.print("Updating Freq for the BioFeedback : ");
      Serial.println(leg->Frequency);
    }



    leg->Heel_Strike_Count = 0;
    leg->Heel_Strike = 0;
  }


  return;
}
