#ifndef UTILITES_HEADER
#define UTILITES_HEADER

class Averager{
private:
  double avg;
  double count;
public:
  void update(double value);
  double getAverage();
  void reset();
};

double clamp(double val, double min, double max);

#endif
