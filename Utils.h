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

void updateMax(double* max, double val);

#endif
