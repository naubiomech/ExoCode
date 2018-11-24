#include <Arduino.h>
#include "Exoskeleton.hpp"
#include "Board.hpp"
#include "Leg.hpp"
#include "Report.hpp"
#include "Receive_and_Transmit.hpp"

Exoskeleton::Exoskeleton(ExoPins* exoPins){
  left_leg = new Leg(exoPins->left_leg);
  right_leg = new Leg(exoPins->right_leg);

  commandSerial = new SoftwareSerial(exoPins->bluetooth_tx, exoPins->bluetooth_rx);
  commandSerial->begin(115200);

  report = generateReport();
}

void Exoskeleton::startTrial(){
  enableExo();
  trialStarted = true;
  reportDataTimer.reset();
  receiveDataTimer.reset();
}

void Exoskeleton::endTrial(){
  disableExo();
  trialStarted = false;
}

void Exoskeleton::run(){
  this->measureSensors();
  //TODO implement error checking
  this->attemptCalibration();
  //TODO Implement the balance baseline
  //TODO apply auto kf here
  this->applyControl();

  this->applyTorque();
}

void Exoskeleton::sendReport(){
  if (trialStarted &&
      (reportDataTimer.check())) {
    reportDataTimer.reset();
    send_report(this);
  }
}

void Exoskeleton::attemptCalibration(){
  left_leg->attemptCalibration();
  right_leg->attemptCalibration();
}

void Exoskeleton::applyControl(){
  left_leg->applyControl();
  right_leg->applyControl();
}

void Exoskeleton::calibrateTorque(){
  left_leg->startTorqueCalibration();
  right_leg->startTorqueCalibration();
  long torque_calibration_value_time = millis();
  while (millis() - torque_calibration_value_time < 1000){
    left_leg->updateTorqueCalibration();
    right_leg->updateTorqueCalibration();
  }
  left_leg->endTorqueCalibration();
  right_leg->endTorqueCalibration();
}

void Exoskeleton::calibrateFSRs(){
  left_leg->calibrateFSRs();
  right_leg->calibrateFSRs();
}

void Exoskeleton::calibrateIMUs(){
  right_leg->calibrateIMUs();
  left_leg->calibrateIMUs();
}

void Exoskeleton::resetStartingParameters(){
  left_leg->resetStartingParameters();
  right_leg->resetStartingParameters();
}

void Exoskeleton::measureSensors(){
	left_leg->measureSensors();
  right_leg->measureSensors();
}

bool Exoskeleton::checkMotorErrors(){
  if (!resetting_motors && (left_leg->checkMotorErrors() || right_leg->checkMotorErrors())){
    disableMotors();
    resetting_motors = true;
    motor_shutdown.reset();
  }

  if (resetting_motors && motor_shutdown.check()){
    enableMotors();
    motor_startup.reset();
  }

  if (resetting_motors && motor_startup.check()){
    resetting_motors = false;
  }

  return resetting_motors;
}

void Exoskeleton::disableMotors(){
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
}

void Exoskeleton::enableMotors(){
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
}

void Exoskeleton::enableExo(){
  enableMotors();
  digitalWrite(LED_PIN, HIGH);
}

void Exoskeleton::disableExo(){
  disableMotors();
  digitalWrite(LED_PIN, LOW);
}

void Exoskeleton::applyTorque(){
  if(!(left_leg->applyTorque() &&
       right_leg->applyTorque())){
    disableExo();
  }
}

void Exoskeleton::receiveMessages(){
  if (receiveDataTimer.check() == 1) {
    receiveDataTimer.reset();
    receive_and_transmit(this);
  }
}

void Exoskeleton::checkReset(){
  if (!trialStarted) {
    resetStartingParameters();
  }
}

ExoReport* Exoskeleton::generateReport(){
  ExoReport* report = new ExoReport();
  fillLocalReport(report);
  report->left_leg = left_leg->generateReport();
  report->right_leg = right_leg->generateReport();
  return report;
}

void Exoskeleton::fillReport(ExoReport* report){
  fillLocalReport(report);
  left_leg->fillReport(report->left_leg);
  right_leg->fillReport(report->right_leg);
}

void Exoskeleton::fillLocalReport(ExoReport* report){

}
