void takeHeelStrikeAngle(Leg* leg) {
  // take the angle during the heel strike

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



void BioFeedback_Baseline(Leg* leg) {
  // calculate the baseline

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
  // update the heel strike mean and apply the biofeedback
  Serial.println("Inside Heel strike");
  if (leg->Heel_Strike_Count >= leg->n_step_biofeedback)
  {
    Serial.println("Inside first if Heel strike");
    leg->Heel_Strike_mean = leg->Heel_Strike / leg->Heel_Strike_Count;

    Serial.print("BioFeedback_Baseline = ");
    Serial.print(leg->Heel_Strike_baseline);
    Serial.print(" ; Difference between mean and desired : ");
    Serial.println(abs(leg->Heel_Strike_mean - leg->BioFeedback_desired));

    //min_knee_error is a dead zone, if the error is minor than something do not apply the control NO BIOFEEDBACK

    if (((abs(leg->Heel_Strike_mean - leg->BioFeedback_desired) < leg->min_knee_error) || (abs(leg->Heel_Strike_mean) < abs(leg->BioFeedback_desired))) && leg->BioFeedback_Baseline_flag == true) {
      leg->NO_Biofeedback = true;
      Serial.println("NO Biofeedback true");
    } else {
      leg->NO_Biofeedback = false;
      Serial.println("NO Biofeedback false");
    }

    if (abs(leg->Heel_Strike_mean - leg->BioFeedback_desired) >= leg->min_knee_error && leg->BioFeedback_Baseline_flag == true)
    {
      Serial.println("Inside second if Heel strike");

      // consider as a max error 20 deg
      // limits the max gain that we can use to modify the frequency
      leg->Biofeedback_ctrl_max = min(abs((20) / (leg->BioFeedback_desired - leg->Heel_Strike_baseline)), 3);



      // Frequency = K error
      // error=(leg->BioFeedback_desired - leg->Heel_Strike_mean);
      // is the gain
      // the bigger is the difference between the baseline and the desired the smaller the gain.
      //map the result in the frequency array .....*(BioFeedback_Freq_max - BioFeedback_Freq_min) + BioFeedback_Freq_max
      //      leg->Frequency = -min(1, abs((leg->BioFeedback_desired - leg->Heel_Strike_mean) / (leg->BioFeedback_desired - leg->Heel_Strike_baseline)) / leg->Biofeedback_ctrl_max) * (BioFeedback_Freq_max - BioFeedback_Freq_min) + BioFeedback_Freq_min;


      leg->Frequency =min(40, abs(leg->BioFeedback_desired - leg->Heel_Strike_mean)) / abs(leg->BioFeedback_desired - leg->Heel_Strike_baseline)* (BioFeedback_Freq_min - BioFeedback_Freq_max)+ BioFeedback_Freq_max;



      Serial.print("Updating Freq for the BioFeedback : ");
      Serial.println(leg->Frequency);
    }



    leg->Heel_Strike_Count = 0;
    leg->Heel_Strike = 0;
  }


  return;
}
