#include "Joint.hpp"
#include "Board.hpp"

Joint::Joint(Motor* motor, TorqueSensor* torque_sensor){
  this->motor = motor;
  this->torque_sensor = torque_sensor;
}

void Joint::measureError(){
  motor->measureError();
}

bool Joint::hasErrored(){
  return motor->hasErrored();
}

bool Joint::applyTorque(){
  if (torque_sensor->getTorque() > 25){
    return false;
  }
  motor->write(motor_output);
  return true;
}

void Joint::changeControl(StateID state_id){
  controller->changeControl(state_id);
}

void Joint::updateMotorOutput(double FSR_percentage, double max_FSR_percentage){
  motor_output = controller->getControlAdjustment(torque_sensor->getTorque(), FSR_percentage, max_FSR_percentage);
}

void Joint::setToZero(){
  controller->setToZero();
}

void Joint::resetStartingParameters(){
  controller->resetStartingParameters();
}

void Joint::adjustShapingForTime(double planter_time){
  this->controller->adjustShapingForTime(planter_time);
}

void Joint::applyAutoKF(){
  controller->applyAutoKF();
}

void Joint::measureTorque(){
  torque_sensor->measureTorque();
}

void Joint::startTorqueCalibration(){
  void setTorqueScalar(double scalar);
  torque_sensor->startTorqueCalibration();
}

void Joint::updateTorqueCalibration(){
  torque_sensor->updateTorqueCalibration();
}

void Joint::endTorqueCalibration(){
  torque_sensor->endTorqueCalibration();
}

void Joint::setSign(int sign){
  motor->setSign(sign);
  torque_sensor->setSign(sign);
}

JointReport* Joint::generateReport(){
  JointReport* report = new JointReport();
  fillLocalReport(report);
  report->motor_report = motor->generateReport();
  report->torque_sensor_report = torque_sensor->generateReport();
  return report;
}

void Joint::fillReport(JointReport* report){
  fillLocalReport(report);
  motor->fillReport(report->motor_report);
  torque_sensor->fillReport(report->torque_sensor_report);
}

void Joint::fillLocalReport(JointReport* report){
  report->pid_setpoint = controller->getLastSetpoint();
}
