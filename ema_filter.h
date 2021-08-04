#ifndef EMA_FILTER_H
#define EMA_FILTER_H

template <typename T>
T ema_with_context(T old_value, T raw, double alpha) {
  T new_value = (alpha * raw) + ((1- alpha) * old_value);
  return new_value;
}

#endif
