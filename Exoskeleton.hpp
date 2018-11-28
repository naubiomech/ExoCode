#ifndef EXOSKELETON_HEADER
#define EXOSKELETON_HEADER
#include "Leg.hpp"
#include <Metro.h>
#include "Report.hpp"
#include "Transceiver.hpp"

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

  bool trialStarted = false;
  Metro reportDataTimer = Metro(10);
  Metro receiveDataTimer = Metro(1);
  ExoReport* report;
  Transceiver* transceiver;

  Leg* left_leg;
  Leg* right_leg;

public:

  Exoskeleton(Leg* left_leg, Leg* right_leg, Transceiver* transceiver);
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
  void startTrial();
  void endTrial();
  void receiveMessages();
  void checkReset();
  void sendReport();
  ExoReport* generateReport();
  void fillReport(ExoReport* report);
};

#endif
