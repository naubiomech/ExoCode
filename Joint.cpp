#include "Joint.hpp"

Joint::Joint(JointPins* pins){
  motor = new Motor(&pins->motor_pins);
  torque_sensor = new TorqueSensor(&pins->torque_sensor_pins);
}

void Joint::measureError(){
  motor->measureError();
}

bool Joint::hasErrored(){
  return motor->hasErrored();
}

bool Joint::applyTorque(int state){
  double torque = torque_sensor->getTorque();
  return motor->applyTorque(state, torque);
}

void Joint::autoKF(int state){
  motor->autoKF(state);
}

void Joint::writeToMotor(int value){
  motor->writeToMotor(value);
}

void Joint::changeState(int state){
  motor->changeState(state);
}

void Joint::updateSetpoint(int state){
  motor->updateSetpoint(state);
}

void Joint::sigmoidCurveSetpoint(int state){
  motor->sigmoidCurveSetpoint(state);
}

void Joint::setToZero(){
  motor->setToZero();
}

void Joint::adjustControl(int state, int state_old, int FSR_baseline_FLAG){
  motor->adjustControl(state, state_old, FSR_baseline_FLAG);
}

void Joint::resetStartingParameters(){
  motor->resetStartingParameters();
}

void Joint::takeBaseline(int state, int state_old, int* FSR_baseline_FLAG){
  motor->takeBaseline(state, state_old, FSR_baseline_FLAG);
}

void Joint::setControlAlgorithm(ControlAlgorithm control_algorithm){
  motor->setControlAlgorithm(control_algorithm);
}

void Joint::adjustShapingForTime(double planterTime){
  motor->adjustShapingForTime(planterTime);
}

void Joint::setTorqueScalar(double scalar){
  motor->setTorqueScalar(scalar);
}

void Joint::applyPlanterControlAlgorithm(bool taking_baseline, double FSR_percentage, double max_FSR_percentage){
  motor->applyPlanterControlAlgorithm(taking_baseline, FSR_percentage, max_FSR_percentage);
}

void Joint::updateKFPIDError(){
  double torque = torque_sensor->getTorque();
  motor->updateKFPIDError(torque);
}

void Joint::applyAutoKF(){
  motor->applyAutoKF();
}

double Joint::getTorque(){
  return torque_sensor->getTorque();
}

void Joint::measureTorque(){
  torque_sensor->measureTorque();
}

void Joint::startTorqueCalibration(){
  torque_sensor->startTorqueCalibration();
}

void Joint::updateTorqueCalibration(){
  torque_sensor->updateTorqueCalibration();
}

void Joint::endTorqueCalibration(){
  torque_sensor->endTorqueCalibration();
}

JointReport Joint::generateReport(){
  JointReport report;
  fillLocalReport(&report);
  report.motor_report = motor->generateReport();
  report.torque_sensor_report = torque_sensor->generateReport();
  return report;
}

void Joint::fillReport(JointReport* report){
  fillLocalReport(report);
  motor->fillReport(&report->motor_report);
  torque_sensor->fillReport(&report->torque_sensor_report);
}

void Joint::fillLocalReport(JointReport* report){

}
