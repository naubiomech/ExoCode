void takeHeelStrikeAngle(Leg* leg) {
  // take the angle during the heel strike

  //leg->Heel_Strike += (pot(leg->Potentiometer_pin) + leg->Biofeedback_bias);                //Take baseline knee angle data
  leg->Heel_Strike_Count++;
  Serial.print("Heel strike sum and n of steps: ");
  Serial.print(leg->Heel_Strike);
  Serial.print(" , ");

  if (leg->BioFeedback_Baseline_flag == true) {
    Heel_strike_update_and_biofeedback(right_leg);
  }
  return;
}


void BioFeedback_Baseline(Leg* leg) {
  // calculate the baseline

  if (leg->BioFeedback_Baseline_flag == false) {
    if (leg->Heel_Strike_Count >= leg->n_step_biofeedback_base)
    {
      Serial.print("UPDATING BASELINE for the BioFeedback : ");
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
  // update the heel strike mean and apply the biofeedback
  Serial.println("Inside Heel strike");

  if (leg->Heel_Strike_Count >= leg->n_step_biofeedback) {
    Serial.println("Inside first if Heel strike");
    leg->Heel_Strike_mean = leg->Heel_Strike / leg->Heel_Strike_Count;
    Serial.print("Heel_Strike_mean = ");
    Serial.print(leg->Heel_Strike_mean);
    Serial.print(" ; Difference between mean and desired : ");
    Serial.println((leg->Heel_Strike_mean - leg->BioFeedback_desired));

    if (leg->Heel_Strike_mean >= leg->BioFeedback_desired) { //added
      leg->NO_Biofeedback = true;
      Serial.println("NO Biofeedback true");
    } else {
      leg->NO_Biofeedback = false;
      Serial.println("NO Biofeedback false");
      Serial.println("Inside second if Heel strike");
      leg->Frequency = map((leg->BioFeedback_desired - leg->Heel_Strike_mean), 100, 0, 200, 1000); //added
      if (leg->Frequency >= 1000) {
        leg->Frequency = 1000;
      } else if (leg->Frequency <= 200) {
        leg->Frequency = 200;
      }
      Serial.print("Updating Freq for the BioFeedback : ");
      Serial.println(leg->Frequency);
    }
    leg->Heel_Strike_Count = 0;
    leg->Heel_Strike = 0;
  }
  return;
}
