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

double Clamp::Clamp(double upper, double lower){
  this.upper = upper;
  this.lower = lower;
}

double Clamp::clamp(double val){
  return min(max(val, lower),upper);
}
