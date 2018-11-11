#ifndef UTILITES_HEADER
#define UTILITES_HEADER

class RunningAverage{
private:
  double avg = 0;
  double count = 0;
public:
  void update(double value);
  double getAverage();
  void reset();
};

class MovingAverage{
private:
  double* previous_values;
  double average;
  int size;
public:
  MovingAverage(int size);
  double update(double value);
  double getAverage();
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
  int crossed_threshold_max;
  int crossed_threshold_count;
  bool state;
public:
  Threshold(bool starting_state, double threshold_value, int crossed_threshold_max);
  bool getState(double value);
};

class Timer{
private:
  double start_time = 0;
  double pause_time = 0;
public:
  Timer();
  double lap();
  double lapSec();
  void reset();
  void pause();
  void resume();
};

class Max{
private:
  double maxVal;
public:
  Max();
  double getMax();
  void update(double value);
  void reset();
};

void updateMax(double* max_val, double val);

#endif
