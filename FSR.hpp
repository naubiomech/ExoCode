#ifndef FSR_HEADER
#define FSR_HEADER
#include "Parameters.hpp"
#include "Utils.hpp"
#include "Report.hpp"
#include "Port.hpp"
#include <vector>

class FSR{
private:
  void fillLocalReport(FSRReport* report);
  InputPort* port;
  double force = 0;

public:
  FSR(InputPort* port);
  void measureForce();
  double getForce();
};

class FSRGroup{
private:
  int fsr_count;
  std::vector<FSR*> fsrs;

  double fsr_percent_thresh = 0.9;
  Max* max_fsr_voltage = new Max();
  Max* max_fsr_percentage = new Max();
  MovingAverage* peak_average = new MovingAverage(FSR_CALIBRATION_PEAK_COUNT);

  double calibration_peak;
  double fsr_percentage;

  void fillLocalReport(FSRReport* report);

public:
  FSRGroup(std::vector<FSR*> fsrs);

  void measureForce();
  double getForce();
  void calibrate();
  double getThreshold();
  void setPercentageThreshold(double percent);
  void resetMaxes();
  void updateMaxes();
  double getPercentage();
  double getMaxPercentage();
  void updateBaseline();
  double force;
  FSRReport* generateReport();
  void fillReport(FSRReport* report);
};
#endif
