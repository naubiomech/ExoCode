//YF
void takestridetime(Leg* leg) { //take the duration of L/R stride
  if (leg->stridelength_target != 0 && leg->stridelength_update >= leg->stridelength_target && FLAG_BIOFEEDBACK) {
    leg->NO_Biofeedback = false;
    leg->score++;
  } else {
    leg->NO_Biofeedback = true;
  }
  leg->Heel_Strike_Count++;
  leg->stridelength_update = 0;
  
  return;
}

void biofeedback_step_baseline(Leg* leg) {
  leg->stridelength_baseline = leg->baseline_value; //or p_steps_l->plant_peak_mean;
  leg->stridelength_target = leg->stridelength_baseline * BF_scale; //subject to change
  leg->BioFeedback_Baseline_flag = true;
  leg->BIO_BASELINE_FLAG = false;

  return;
}

void biofeedback_step_update (Leg* leg) {
  if (leg->state == 3) {
    if (leg->stridelength_update < leg->FSR_Toe_Average) {
      leg->stridelength_update = leg->FSR_Toe_Average;
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
