#include <Arduino.h>
#include "Leg.hpp"
#include "Pins.hpp"
#include "Shaping_Functions.hpp"
#include "Motor.hpp"
#include "IMU.hpp"
#include "Report.hpp"

Leg::Leg(LegPins* legPins){
  this->foot_fsrs = new FSRGroup(legPins->fsr_pins, legPins->fsr_count);
  this->joint_count = legPins->joint_count;
  this->joints = new Joint*[joint_count];
  this->fsr_group_count = 1;
  this->fsrs = new FSRGroup*[this->fsr_group_count];
  this->fsrs[0] = foot_fsrs;
  for (int i = 0; i < legPins->joint_count; i++){
    JointPins* joint_pins = &(legPins->joint_pins[i]);
    this->joints[i] = new Joint(joint_pins);
  }
  for (int i = 0; i < legPins->imu_count; i++){
    IMUPins* imu_pins = &(legPins->imu_pins[i]);
    this->imus[i] = new IMU(imu_pins);
  }
}

double Leg::getBalanceReference(){
  return foot_fsrs->getBalanceReference() * Prop_Gain;
}

void Leg::calibrateFSRs(){
  foot_fsrs->calibrate();
}

void Leg::startTorqueCalibration(){
  for(int i = 0; i < joint_count;i++){
    joints[i]->startTorqueCalibration();
  }
}

void Leg::updateTorqueCalibration(){
  for(int i = 0; i < joint_count;i++){
    joints[i]->updateTorqueCalibration();
  }
}

void Leg::endTorqueCalibration(){
  for(int i = 0; i < joint_count;i++){
    joints[i]->endTorqueCalibration();
  }
}

bool Leg::checkMotorErrors(){
  for(int i = 0; i < joint_count;i++){
    if(joints[i]->hasErrored()){
      return true;
    }
  }
  return false;
}

void Leg::autoKF(){
  for(int i = 0; i < joint_count;i++){
    joints[i]->autoKF(state);
  }
}

void Leg::resetStartingParameters(){
  this->activate_in_3_steps = 1;

  this->planter_step_count = 0;
  this->first_step = 1;

  for (int i = 0; i < joint_count; i++){
    joints[i]->resetStartingParameters();
  }
}

void Leg::setZeroIfSteadyState(){
  if (isSteadyState()){
    setToZero();
  }
}

bool Leg::isSteadyState(){
  return this->state_time->lap() > STEADY_STATE_TIMEOUT;
}

void Leg::startIncrementalActivation(){
  this->increment_activation_starting_step = 0;
}

void Leg::updateIncrementalActivation(){
  double since_activation_step_count = this->planter_step_count - this->increment_activation_starting_step;
  double activation_percent = since_activation_step_count / ACTIVATION_STEP_COUNT;
  for(int i = 0; i < joint_count; i++){
    joints[i]->setTorqueScalar(activation_percent);
  }
}

int Leg::determineState(boolean foot_on_ground){
  int new_state;
  if (swing_state_threshold->getState((double) foot_on_ground)){
    new_state = SWING;
  } else {
    new_state = LATE_STANCE;
  }

  return new_state;
}

void Leg::changeState(int new_state, int old_state){
  this->state = new_state;
  this->state_time->reset();

  for(int i = 0; i < joint_count; i++){
    joints[i]->changeState(state);
  }

  Phase new_phase = determinePhase(new_state, old_state, this->current_phase);
  if (new_phase != this->current_phase){
    this->current_phase = new_phase;
    this->changePhase(new_phase);
  }
}

void Leg::changePhase(int new_phase){
  if (new_phase == PLANTER){
    this->triggerPlanterPhase();
  } else if (new_phase == DORSI){
    this->triggerDorsiPhase();
  }
}

void Leg::adjustControl(){
  applyControlAlgorithm();
}

void Leg::applyControlAlgorithm(){
  if (this->current_phase == PLANTER){
    this->applyPlanterControlAlgorithm();
  } else if (this->current_phase == DORSI){
    this->applyDorsiControlAlgorithm();
  }
}

void Leg::applyPlanterControlAlgorithm(){
  for(int i =0; i < fsr_group_count; i++){
    fsrs[i]->updateMaxes();
  }

  double FSR_percentage = foot_fsrs->getPercentage();
  double max_FSR_percentage = foot_fsrs->getMaxPercentage();
  bool taking_baseline = false;

  for(int i = 0; i < fsr_group_count; i++){
    fsrs[i]->updateMaxes();
  }

  for(int i = 0; i < joint_count; i++){
    joints[i]->applyPlanterControlAlgorithm(FSR_percentage, taking_baseline, max_FSR_percentage);
  }
}

void Leg::applyDorsiControlAlgorithm(){
  for(int i = 0; i < joint_count; i++){
    joints[i]->applyAutoKF();
  }
}

Phase Leg::determinePhase(int new_state, int old_state, Phase current_phase){
  double current_phase_time;
  Phase next_phase;
  if (new_state == SWING && old_state == LATE_STANCE && current_phase == DORSI){
    next_phase = PLANTER;
    current_phase_time = planter_timer->lap();
  } else if (new_state == LATE_STANCE && old_state == SWING && current_phase == PLANTER){
    next_phase = DORSI;
    current_phase_time = dorsi_timer->lap();
  } else {
    return current_phase;
  }

  if (current_phase_time <=step_time_length /4){
    return current_phase;
  }

  return next_phase;
}

bool Leg::determine_foot_on_ground(){
  boolean foot_on_fsr = this->foot_fsrs->getForce() > this->foot_fsrs->getThreshold();
  return foot_on_fsr;
}

void Leg::triggerPlanterPhase(){
  this->planter_timer->reset();

  for (int i = 0; i <fsr_group_count;i++){
    fsrs[i]->resetMaxes();
  }
}

void Leg::triggerDorsiPhase(){
  this->planter_step_count++; // you have accomplished a step
  this->dorsi_timer->reset();

  double planter_time = this->planter_timer->lap();
  this->planter_mean = this->planter_time_averager->update(planter_time);

  for (int i =0; i < joint_count; i++){
    joints[i]->adjustShapingForTime(planter_time);
  }
}

int Leg::takeBaselineTriggerDorsiStart(){

  double planter_time = this->planter_timer->lap();
  this->planter_mean = this->planter_time_averager->update(planter_time);

  for(int i = 0; i < fsr_group_count;i++){
    fsrs[i]->updateBaseline();
  }

  if (this->planter_step_count_base >= n_step_baseline){
    this->planter_step_count_base = 0;

    return 0;
  }

  return 1;
}

int Leg::takeBaselineTriggerPlanterStart(){

  double dorsi_time = this->dorsi_timer->lap();
  this->dorsi_mean = this->dorsi_time_averager->update(dorsi_time);

  return 1;
}

void Leg::triggerLateStance(){
  if (this->set_motors_to_zero_torque){
    setToZero();
    this->set_motors_to_zero_torque = false;
  }
}

void Leg::triggerSwing(){

}

void Leg::setToZero(){
  for(int i = 0; i<joint_count; i++){
    joints[i]->setToZero();
  }
}

void Leg::applyStateMachine(){
  bool foot_on_ground = determine_foot_on_ground();
  int new_state = this->determineState(foot_on_ground);
  if (new_state != this->state){
    changeState(new_state, this->state);
  }
  updateIncrementalActivation();
  for(int i = 0; i < joint_count;i++){
    joints[i]->updateSetpoint(state);
  }
}

void Leg::measureSensors(){
  for(int i = 0; i < joint_count; i++){
    this->joints[i]->measureTorque();
    this->joints[i]->measureError();
  }
  this->foot_fsrs->measureForce();

}

void Leg::takeFSRBaseline(){
  if (FSR_baseline_FLAG){
    for (int i = 0; i < joint_count; i++){
      joints[i]->takeBaseline(state, state_old, &FSR_baseline_FLAG);
    }
  }
}

bool Leg::applyTorque(){
  for(int i = 0; i < joint_count; i++){
    if (!joints[i]->applyTorque(state)){
      state = 9;
      return false;
    }
  }
  return true;
}

void Leg::calibrateIMUs(){
  for(int i = 0; i < imu_count; i++){
    imus[i]->calibrate();
  }
}

void Leg::measureIMUs(){
  for(int i = 0; i < imu_count; i++){
    imus[i]->measure();
  }
}

LegReport* Leg::generateReport(){
  LegReport* report = new LegReport(joint_count, fsr_group_count);
  fillLocalReport(report);
  for (int i = 0; i < joint_count; i++){
    report->joint_reports[i] = joints[i]->generateReport();
  }
  for (int i = 0; i < fsr_group_count; i++){
    report->fsr_reports[i] = fsrs[i]->generateReport();
  }
  return report;
}

void Leg::fillReport(LegReport* report){
  fillLocalReport(report);
  for (int i = 0; i < joint_count; i++){
    joints[i]->fillReport(&(report->joint_reports[i]));
  }
  for (int i = 0; i < fsr_group_count; i++){
    fsrs[i]->fillReport(&(report->fsr_reports[i]));
  }
}

void Leg::fillLocalReport(LegReport* report){
	report->state = state;
	report->phase = current_phase;
}
