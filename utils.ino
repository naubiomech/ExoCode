
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

double clamp(double val, double lower, double upper){
  return min(max(val, lower),upper);
}
