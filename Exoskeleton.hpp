#ifndef EXOSKELETON_HEADER
#define EXOSKELETON_HEADER
#include "Leg.hpp"
#include "Pins.hpp"
#include <Metro.h>
#include "Report.hpp"

class Exoskeleton{
private:
  void fillLocalReport(ExoReport* report);
  void disableMotors();
  void enableMotors();
  void attemptCalibration();
  void applyControl();

  bool resetting_motors = false;
  Metro motor_shutdown = Metro(16);
  Metro motor_startup = Metro(8);

  Leg* left_leg;
  Leg* right_leg;

public:
  Exoskeleton(ExoPins* exoPins);
  void run();
  void measureSensors();
  bool checkMotorErrors();
  void disableExo();
  void enableExo();
  void applyTorque();
  void adjustControl();
  void resetStartingParameters();
  void calibrateTorque();
  void calibrateFSRs();
  void setZeroIfStateState();
  void calibrateIMUs();
  ExoReport* generateReport();
  void fillReport(ExoReport* report);
};

#endif
