#ifndef FSR_HEADER
#define FSR_HEADER
#include "Pins.hpp"
#include "Utils.hpp"
#include "Report.hpp"

class FSR{
private:
  bool isCalibrating = false;
  unsigned long fsrCalibrationStartTime;
  void startCalibration();
  void updateCalibration();
  void endCalibration();
  void fillLocalReport(FSRReport* report);

public:
  FSR(int pin);
  double calibrate();
  void measureForce();
  double getForce();
  double getBalanceReference();

  unsigned int pin;

  double force = 0;
  double peak_force = 0;

  double fsr = 0;
  int address_FSR;
  int baseline_address;
  double baseline_value;
};

class FSRGroup{
private:
  int fsr_count;
  double calibration_peak;
  double fsr_percent_thresh = 0.9;
  FSR** fsrs;
  Max* max_fsr_voltage = new Max();
  Max* max_fsr_percentage = new Max();
  double plant_peak_mean = 0;
  RunningAverage* plant_peak_averager = new RunningAverage();

  double fsr_percentage;

  void startCalibration();
  void updateCalibration();
  void fillLocalReport(FSRReport* report);

public:
  FSRGroup(FSRGroupPins* fsr_pins);

  void measureForce();
  double getForce();
  void calibrate();
  double getBalanceReference();
  double getThreshold();
  void setPercentageThreshold(double percent);
  void resetMaxes();
  void updateMaxes();
  double getPercentage();
  double getMaxPercentage();
  void startBaseline();
  void updateBaseline();
  double force;
  FSRReport generateReport();
  void fillReport(FSRReport* report);
};
#endif
