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

Threshold::Threshold(double threshold, int crossed_threshold_max){
  this->upper_threshold = upper_threshold;
  this->crossed_threshold_max = crossed_threshold_max;
  this->state = 0;
  this->crossed_threshold_count = 0;
}

bool Threshold::confirmState(double value){
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
