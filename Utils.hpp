#ifndef UTILITES_HEADER
#define UTILITES_HEADER

class RunningAverage{
private:
  double avg;
  double count;
public:
  RunningAverage();
  RunningAverage(double initial);
  double update(double value);
  double getAverage();
  void reset();
  void reset(double value);
};

class MovingAverage{
private:
  double* previous_values;
  double total;
  double average;
  int index;
  int size;
public:
  MovingAverage(int size);
  ~MovingAverage();
  double update(double value);
  double getAverage();
  void reset();
};

class Clamp{
private:
  double upper;
  double lower;
public:
  Clamp(double lower, double upper);
  double clamp(double value);
};

class Threshold{
private:
  double upper_threshold;
  double lower_threshold;
  bool state;
public:
  Threshold(bool starting_state, double upper_threshold_value, double lower_threshold_value);
  bool getState(double value);
  void setUpperThreshold(double value);
  void setLowerThreshold(double value);
};

class Timer{
private:
  unsigned long start_time;
  unsigned long pause_time;
public:
  Timer();
  double lap();
  double lapSec();
  void reset();
  void pause();
  void resume();
};

class Chrono{
private:
  unsigned long start_time, interval_time;
public:
  Chrono(unsigned long interval);
  bool check();
  void reset();
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

class Min{
private:
  double minVal;
public:
  Min();
  double getMin();
  void update(double value);
  void reset();
};

class ChangeTrigger{
private:
  bool state;
public:
  ChangeTrigger(bool start_state);
  bool update(bool state);
};


class Range{
private:
  Max* maximum;
  Min* minimum;
  RunningAverage* min_avg;
  RunningAverage* max_avg;
  double threshold;
  double calculateThreshold();
public:
  Range(double first_min, double first_max);
  double getMax();
  double getMin();
  void update(double value);
  void reset(double min, double max);
  void reset();
};

void updateMax(double* max_val, double val);

#endif
