#include <Arduino.h>
#include "Leg.hpp"
#include "Pins.hpp"
#include "Shaping_Functions.hpp"
#include "Motor.hpp"
#include "IMU.hpp"
#include "Report.hpp"

Leg::Leg(State* states, std::vector<Joint*> joints, std::vector<FSRGroup*> fsrs, std::vector<IMU*> imus){
  state = states;
  this->joints.size() = joints.size();
  this->imus.size() = imus.size();
  this->fsrs.size() = fsrs.size();

  this->joints = joints;
  this->fsrs = fsrs;
  this->imus = imus;

  this->foot_fsrs = fsrs[0];
}

void Leg::attemptCalibration(){
  // TODO add any flag dependent calibrations here
}

void Leg::applyControl(){
  this->applyStateMachine();
  this->adjustControl();
}

void Leg::calibrateFSRs(){
  foot_fsrs->calibrate();
}

void Leg::startTorqueCalibration(){
  for(int i = 0; i < joints.size();i++){
    joints[i]->startTorqueCalibration();
  }
}

void Leg::updateTorqueCalibration(){
  for(int i = 0; i < joints.size();i++){
    joints[i]->updateTorqueCalibration();
  }
}

void Leg::endTorqueCalibration(){
  for(int i = 0; i < joints.size();i++){
    joints[i]->endTorqueCalibration();
  }
}

bool Leg::checkMotorErrors(){
  for(int i = 0; i < joints.size();i++){
    if(joints[i]->hasErrored()){
      return true;
    }
  }
  return false;
}

void Leg::resetFSRMaxes(){
  for (int i = 0; i <fsrs.size();i++){
    fsrs[i]->resetMaxes();
  }
}

void Leg::adjustJointSetpoints(){
  double FSR_percentage = foot_fsrs->getPercentage();
  double max_FSR_percentage = foot_fsrs->getMaxPercentage();

  for(int i = 0; i < joints.size(); i++){
    joints[i]->updateMotorOutput(FSR_percentage, max_FSR_percentage);
  }
}

void Leg::runAutoKF(){
  for(int i = 0; i < joints.size(); i++){
    joints[i]->applyAutoKF();
  }
}

void Leg::resetStartingParameters(){
  for (int i = 0; i < joints.size(); i++){
    joints[i]->resetStartingParameters();
  }
}

void Leg::adjustShapingForTime(double time){
  for (int i =0; i < joints.size(); i++){
    joints[i]->adjustShapingForTime(time);
  }
}

void Leg::setZeroIfSteadyState(){
  if (isSteadyState()){
    setToZero();
  }
}

bool Leg::isSteadyState(){
  return state->getStateTime() > STEADY_STATE_TIMEOUT;
}

void Leg::incrementStepCount(){
  step_count++;
}

void Leg::startIncrementalActivation(){
  this->increment_activation_starting_step = step_count;
}

bool Leg::hasStateChanged(boolean foot_on_ground){
  return  swing_state_threshold->getState((double) foot_on_ground) &&
    state->getStateTime() <= step_time_length / 4;
}

void Leg::changeState(){
  state->changeState();
  state->setContext(this);
}

void Leg::adjustControl(){
  this->state->run();
  this->adjustJointSetpoints();
}

void Leg::changeJointControl(StateID state_id){
  for(int i = 0; i < joints.size();i++){
    joints[i]->changeControl(state_id);
  }
}

bool Leg::determine_foot_on_ground(){
  boolean foot_on_fsr = this->foot_fsrs->getForce() > this->foot_fsrs->getThreshold();
  return foot_on_fsr;
}

void Leg::setZeroIfNecessary(){
  if (this->set_motors_to_zero_torque){
    setToZero();
    this->set_motors_to_zero_torque = false;
  }
}

void Leg::setToZero(){
  for(int i = 0; i<joints.size(); i++){
    joints[i]->setToZero();
  }
}

void Leg::applyStateMachine(){
  bool foot_on_ground = determine_foot_on_ground();
  if (this->hasStateChanged(foot_on_ground)){
    changeState();
  } else {
    setZeroIfSteadyState();
  }
}

void Leg::measureSensors(){
  for(int i = 0; i < joints.size(); i++){
    this->joints[i]->measureTorque();
    this->joints[i]->measureError();
  }
  this->foot_fsrs->measureForce();
  this->measureIMUs();

}

bool Leg::applyTorque(){
  for(int i = 0; i < joints.size(); i++){
    if (!joints[i]->applyTorque()){
      return false;
    }
  }
  return true;
}

void Leg::calibrateIMUs(){
  for (int i = 0; i < imus.size(); i++){
    imus[i]->calibrate();
  }
}

void Leg::measureIMUs(){
  for (int i = 0; i < imus.size(); i++){
    imus[i]->measure();
  }
}

void Leg::setSign(int sign){
  if (sign == 0){
    return;
  }

  sign = sign / abs(sign);

  for (int i = 0; i < joints.size(); i++){
    joints[i]->setSign(sign);
  }
}

LegReport* Leg::generateReport(){
  LegReport* report = new LegReport(joints.size(), fsrs.size(), imus.size());
  fillLocalReport(report);
  for (int i = 0; i < joints.size(); i++){
    report->joint_reports[i] = joints[i]->generateReport();
  }
  for (int i = 0; i < fsrs.size(); i++){
    report->fsr_reports[i] = fsrs[i]->generateReport();
  }
  for (int i = 0; i < imus.size(); i++){
    report->imu_reports[i] = imus[i]->generateReport();
  }
  return report;
}

void Leg::fillReport(LegReport* report){
  fillLocalReport(report);
  for (int i = 0; i < joints.size(); i++){
    joints[i]->fillReport(report->joint_reports[i]);
  }
  for (int i = 0; i < fsrs.size(); i++){
    fsrs[i]->fillReport(report->fsr_reports[i]);
  }
  for (int i = 0; i < imus.size(); i++){
    imus[i]->fillReport(report->imu_reports[i]);
  }
}

void Leg::fillLocalReport(LegReport* report){
  report->state = state->getStateType();
}
