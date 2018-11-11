#include <Arduino.h>
#include "Utils.hpp"
#include <float.h>

void RunningAverage::update(double value){
  avg += value;
  count++;
}

double RunningAverage::getAverage(){
  return avg / count;
}

void RunningAverage::reset(){
  avg = 0;
  count = 0;
}

MovingAverage::MovingAverage(int size){
  previous_values = new double[size];
  this->size = size;
  this->average = 0;
}

double MovingAverage::update(double value){
  average = value;
  for(int i = 0; i < size-1;i++){
    average += previous_values[i];
    previous_values[i+1] = previous_values[i];
  }
  previous_values[0] = value;
  average /= size;
  return average;
}

double MovingAverage::getAverage(){
  return average;
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

Timer::Timer(){
  start_time = millis();
}

double Timer::lapSec(){
  return this->lap() / 1000.0;
}

double Timer::lap(){
  return millis() - start_time;
}

void Timer::reset(){
  start_time = millis();
}

void Timer::pause(){
  pause_time = millis();
}

void Timer::resume(){
  start_time -= (millis() - pause_time);
}

Max::Max(){
  this->reset();
}

double Max::getMax(){
  return this->maxVal;
}

void Max::update(double value){
  maxVal = max(value, maxVal);
}

void Max::reset(){
  maxVal = FLT_MIN;
}
