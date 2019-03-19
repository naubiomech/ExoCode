#ifndef EXOSKELETON_HEADER
#define EXOSKELETON_HEADER
#include "Arduino.hpp"
#include "Leg.hpp"
#include "Report.hpp"
#include "Communications.hpp"
#include "Utils.hpp"
#include "Message.hpp"

class Exoskeleton{
private:
  void fillLocalReport(ExoReport* report);
  void disableMotors();
  void enableMotors();
  void attemptCalibration();
  void applyControl();

  bool resetting_motors = false;
  Chrono motor_shutdown = Chrono(16);
  Chrono motor_startup = Chrono(8);

  bool trialStarted = false;
  Chrono reportDataTimer = Chrono(10);
  Chrono receiveDataTimer = Chrono(1);
  ExoReport* report;
  Communications* comms;
  OutputPort* motor_enable_port;
  OutputPort* led_port;

  Leg* left_leg;
  Leg* right_leg;

public:

  Exoskeleton(Leg* left_leg, Leg* right_leg, Communications* comms,
              OutputPort* motor_enable_port, OutputPort* led_port);
  ~Exoskeleton();
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
  void processMessage(ExoMessage* msg);
  ExoReport* generateReport();
  void fillReport(ExoReport* report);
};

#endif
