void takestridetime(Leg* leg) { //take the duration of L/R stride
  leg->Heel_Strike_Count++;
  if (leg->Heel_Strike_Count == 1) {
    leg->HS1 = millis();
  }
  if (leg->Heel_Strike_Count == 3) {
    leg->HS3 = millis();
  }
  if (leg->Heel_Strike_Count == 4) {
    leg->HS4 = millis();
  }
  if (leg->Heel_Strike_Count > 4) {
    leg->Heel_Strike_Count = 1;
    leg->HS1=0;
    leg->HS3=0;
    leg->HS4=0;
  }
  return;
}

void biofeedback_step_baseline(Leg* leg) {
if (leg->HS1==0 || leg->HS4==0) {
} else {
  leg->stridetime_baseline = (leg->HS4 - leg->HS1) / 3 / 1000; //unit:s, average for 3 steps
  leg->stridelength_baseline = treadmill_speed * leg->stridetime_baseline / 1000; //unit:m
  leg->BioFeedback_Baseline_flag = true;
  leg->BIO_BASELINE_FLAG = false;
}
  return;
}

void biofeedback_step_update (Leg* leg) {
if (leg->HS1==0 || leg->HS4==0) {
} else {
  leg->stridetime_update = (leg->HS4 - leg->HS1) / 3 / 1000; //unit:s,average for 2 steps
  leg->stridelength_update = treadmill_speed * leg->stridetime_update / 1000; //unit:m
  leg->stridetime_target = leg->stridetime_baseline * 1.25; //subject to change

  if (leg->stridetime_update < leg->stridetime_target) {
    leg->NO_Biofeedback = true;
  } else {
    leg->NO_Biofeedback = false;
    leg->Frequency = 500;
  }
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
