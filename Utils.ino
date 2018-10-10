#include "Utils.h"

void Average::update(double value){
  avg += value;
  count++;
}

double Average::getAverage(){
  return avg / count;
}

void Average::reset(){
  avg = 0;
  count = 0;
}

Clamp::Clamp(double upper, double lower){
  this->upper = upper;
  this->lower = lower;
}

double Clamp::clamp(double val){
  return min(max(val, lower),upper);
}

void updateMax(double* max_value, double value){
  if (*max_value > value) {
    *max_value = value;
  }
}

Threshold::Threshold(bool starting_state, double threshold_value, int crossed_threshold_max){
  this->upper_threshold = threshold_value;
  this->crossed_threshold_max = crossed_threshold_max;
  this->state = starting_state;
  this->crossed_threshold_count = 0;
}

bool Threshold::getState(double value){
  if((value > upper_threshold) == state){
    crossed_threshold_count = 0;
  } else {
    crossed_threshold_count++;
  }

  if(crossed_threshold_count == crossed_threshold_max){
    state = !state;
    crossed_threshold_count = 0;
  }

  return state;
}
