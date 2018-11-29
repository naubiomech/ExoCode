#ifndef FSR_HEADER
#define FSR_HEADER
#include "Parameters.hpp"
#include "Utils.hpp"
#include "Report.hpp"
#include "Port.hpp"
#include "Linked_List.hpp"

class FSR{
private:
  void updateForce(double force);
  void fillLocalReport(FSRReport* report);
  InputPort* port;
  double calibration_peak = 1.0;
  Max* max_force = new Max();
  MovingAverage* peak_average = new MovingAverage(FSR_CALIBRATION_PEAK_COUNT);

  double force = 0;

public:
  FSR(InputPort* port);
  void measureForce();
  double getForce();
  void calibrate();
  void resetMaxes();
};

class FSRGroup{
private:
  int fsr_count;
  LinkedList<FSR*> fsrs;

  double force = 0;

  double fsr_percent_thresh = 0.9;
  Threshold* activation_threshold;
  bool is_activated = false;

  void fillLocalReport(FSRReport* report);

public:
  FSRGroup(LinkedList<FSR*> fsrs);

  bool isActivated();
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
  FSRReport* generateReport();
  void fillReport(FSRReport* report);
};
#endif
