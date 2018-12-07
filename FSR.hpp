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
  double calibration_peak;
  Max* max_force;
  MovingAverage* peak_average;
  Clamp* force_clamp;

  double force;

public:
  FSR(InputPort* port);
  ~FSR();
  void measureForce();
  double getForce();
  void calibrate();
  void resetMaxes();
};

class FSRGroup{
private:
  int fsr_count;
  LinkedList<FSR*> fsrs;

  double force;

  double fsr_percent_thresh;
  Threshold* activation_threshold;
  bool is_activated;

  void fillLocalReport(FSRReport* report);

public:
  FSRGroup(LinkedList<FSR*>* fsrs);
  ~FSRGroup();

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
