
void Averager::update(double value){
  avg += value;
  count++;
}

double Averager::getAverage(){
  return avg / count;
}

void Averager::reset(){
  avg = 0;
  count = 0;
}

double clamp(double val, double lower, double upper){
  return min(max(val, lower),upper);
}
