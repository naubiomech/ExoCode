#ifndef EMA_FILTER_H
#define EMA_FILTER_H

#define LOW_ALPHA   0.001
#define MED_ALPHA   0.01
#define HIGH_ALPHA  0.1

/* Params: Leg to pass into sampler function, old filter value, alpha for filter, function that returns data that you would like to sample */
float ema_with_sampler(Leg* leg, float old_value, float alpha, float (*sampler)(Leg*)) {
  float raw = (*sampler)(leg);
  float new_value = (alpha * raw) + ((1 - alpha) * old_value);
  return new_value;
}

template <typename T>
T ema_with_context(T old_value, T raw, double alpha) {
  T new_value = (alpha * raw) + ((1- alpha) * old_value);
  return new_value;
}

#endif
