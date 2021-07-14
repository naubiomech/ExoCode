#ifndef EMA_FILTER_H
#define EMA_FILTER_H

#define LOW_ALPHA   0.10
#define MED_ALPHA   0.40
#define HIGH_ALPHA  0.95

/* Params: Leg to pass into sampler function, old filter value, alpha for filter, function that returns data that you would like to sample */
float ema_with_sampler(Leg* leg, float old_value, float alpha, float (*sampler)(Leg*)) {
  float raw = (*sampler)(leg);
  float new_value = (alpha * raw) + ((1 - alpha) * old_value);
  return new_value;
}

double ema_with_context(double old_value, double raw, double alpha) {
  double new_value = (alpha * raw) + ((1- alpha) * old_value);
  return new_value;
}

#endif
