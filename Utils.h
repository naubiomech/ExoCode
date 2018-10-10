#ifndef UTILITES_HEADER
#define UTILITES_HEADER

class Average{
private:
  double avg;
  double count;
public:
  void update(double value);
  double getAverage();
  void reset();
};

class Clamp{
private:
  double upper;
  double lower;
public:
  Clamp(double upper, double lower);
  double clamp(double value);
};

class Threshold{
private:
  double upper_threshold;
  int crossed_count;
  bool state;
public:
  Threshold(double upper_threshold);
  bool confirmState(double value);
}

void updateMax(double* max_val, double val);

#endif