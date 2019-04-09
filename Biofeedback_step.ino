void takestridetime(Leg* leg) { //take the duration of L/R stride
  leg->Heel_Strike_Count++;

  if (leg->Heel_Strike_Count > 4) {
    leg->Heel_Strike_Count = 0;
  }
  return;
}

void biofeedback_step_baseline(Leg* leg) {
  takestridetime(leg);
  if (leg->Heel_Strike_Count = 1) {
    leg->HS1 = millis();
  } else if (leg->Heel_Strike_Count = 4) {
    leg->HS4 = millis();
  }
  leg->stridetime_baseline = (leg->HS4 - leg->HS1) / 3 / 1000; //unit:s
  leg->stridelength_baseline = treadmill_speed * leg->stridetime / 1000; //unit:m
  leg->BioFeedback_Baseline_flag = true;
  leg->BIO_BASELINE_FLAG = false;
  return;
}

void biofeedback_step_update (Leg* leg) {
  takestridetime(leg);
  if (leg->Heel_Strike_Count = 1) {
    leg->HS1 = millis();
  } else if (leg->Heel_Strike_Count = 4) {
    leg->HS4 = millis();
  }
  leg->stridetime_update = (leg->HS4 - leg->HS1) / 3 / 1000; //unit:s
  leg->stridelength_update = treadmill_speed * leg->stridetime_update / 1000; //unit:m
  leg->stridetime_target = leg->stridetime_baseline * 1.25; //subject to change

  if (leg->stridetime_update < leg->stridetime_target) {
    leg->NO_Biofeedback = true;
  } else {
    leg->NO_Biofeedback = false;
    leg->Frequency = 500;
  }
  return;
}

void biofeedback_step_state(Leg* leg) {
  if (leg->BioFeedback_Baseline_flag = true) {
    biofeedback_step_baseline(leg);
  } else {
    biofeedback_step_baseline(leg);
  }
  return;
}
