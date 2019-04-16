//YF
void takestridetime(Leg* leg) { //take the duration of L/R stride
  leg->Heel_Strike_Count++;
  if (leg->Heel_Strike_Count == 1) {
    leg->HS1 = millis();
    leg->HS4 = 0;
  }
  if (leg->Heel_Strike_Count == 2) {
    leg->HS2 = millis();
  }
  if (leg->Heel_Strike_Count == 4) {
    leg->HS4 = millis();
  }
  if (leg->Heel_Strike_Count > 4) {
    leg->Heel_Strike_Count = 0;
    leg->HS1 = 0;
    leg->HS2 = 0;
    leg->HS4 = 0;
  }
  return;
}

void biofeedback_step_baseline(Leg* leg) {
  if (leg->HS1 == 0 || leg->HS4 == 0) {
  } else {
    leg->stridetime_baseline = (leg->HS4 - leg->HS1) / 3 / 1000; //unit:s, average for 3 steps
    leg->stridelength_baseline = treadmill_speed * leg->stridetime_baseline*100; //unit:cm
    leg->BioFeedback_Baseline_flag = true;
    leg->BIO_BASELINE_FLAG = false;
  }
  return;
}

void biofeedback_step_update (Leg* leg) {
  if (leg->HS1 == 0 || leg->HS2 == 0) {
  } else {
    leg->stridetime_update = (leg->HS2 - leg->HS1) / 1000; //unit:s,1 step length
    leg->stridelength_update = treadmill_speed * leg->stridetime_update*100; //unit:cm
    leg->stridelength_target = leg->stridelength_baseline * 1.1; //subject to change

    if (leg->stridelength_target!=0 &&leg->stridelength_update >= leg->stridelength_target) {
      leg->NO_Biofeedback = false;
      leg->Frequency = 100;
      leg->score++;
    } else {
      leg->NO_Biofeedback = true;
    }
    
    leg->Heel_Strike_Count = 0;
    leg->HS1 = 0;
    leg->HS2 = 0;
  }
  return;
}

void biofeedback_step_state(Leg* leg) {
  if (leg->BIO_BASELINE_FLAG) {
    biofeedback_step_baseline(leg);
  } else {
    biofeedback_step_update(leg);
  }
  return;
}
