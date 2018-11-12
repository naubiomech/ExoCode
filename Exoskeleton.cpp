#include <Arduino.h>
#include "Exoskeleton.hpp"
#include "Board.hpp"
#include "Leg.hpp"
#include "Report.hpp"

Exoskeleton::Exoskeleton(ExoPins* exoPins){
  left_leg = new Leg(exoPins->left_leg);
  right_leg = new Leg(exoPins->right_leg);
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

void Exoskeleton::resetStartingParameters(){
  left_leg->resetStartingParameters();
  right_leg->resetStartingParameters();
}

void Exoskeleton::adjustControl(){
  left_leg->adjustControl();
  right_leg->adjustControl();
}

void Exoskeleton::setZeroIfSteadyState(){
  left_leg->setZeroIfSteadyState();
  right_leg->setZeroIfSteadyState();
}

void Exoskeleton::applyStateMachine(){
  left_leg->applyStateMachine();
  right_leg->applyStateMachine();

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

void Exoskeleton::checkIMUs(){
  if (BnoControl.check()){
    // TODO Put call to leg imus here
    BnoControl.reset();
  }
}

void Exoskeleton::calibrateIMUs(){
  right_leg->calibrateIMUs();
  left_leg->calibrateIMUs();
}

void Exoskeleton::measureIMUs(){
  right_leg->measureIMUs();
  left_leg->measureIMUs();
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
