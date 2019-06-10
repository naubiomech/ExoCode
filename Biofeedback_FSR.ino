//YF
//void takestrideFSR(Leg* leg) { //take the duration of L/R stride
//  if (leg->BIO_BASELINE_FLAG) {
//    biofeedback_FSR_baseline(leg);
//  } else {
//    biofeedback_FSR_update(leg);
//  }
//  return;
//}

void biofeedback_FSR_baseline(Leg* leg) {

  leg->stridelength_target = leg->p_steps->plant_peak_mean * treadmill_speed;

  leg->BioFeedback_Baseline_flag = true;
  // leg->BIO_BASELINE_FLAG = false;
  return;
}

void biofeedback_FSR_update(Leg* leg) {

  if (leg->state == 3) {
    if (leg->stridelength_update < leg->FSR_Toe_Average / leg->stridelength_target) {
      leg->stridelength_update = leg->FSR_Toe_Average / leg->stridelength_target;
    } else {
      leg->stridelength_update = leg->stridelength_update;
    }
  } else {
    if (leg->state_count_13 > 1) {
      leg->stridelength_update = 0;
    } else {
      leg->stridelength_update = leg->stridelength_update;
    }
  }
  return;
}
