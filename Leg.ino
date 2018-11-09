#include "Leg.h"
#include "Pins.h"
#include "Shaping_Functions.h"

Leg::Leg(LegPins* legPins){
  this->foot_fsrs = new FSRGroup(legPins->fsr_pins, legPins->fsr_count);
  this->motor_count = legPins->motor_count;
  this->motors = new Motor*[motor_count];
  for (int i = 0; i < legPins->motor_count; i++){
    this->motors[i] = new Motor(&legPins->motor_pins[i]);
  }
}

double Leg::getBalanceReference(){
  return foot_fsrs->getBalanceReference() * Prop_Gain;
}

void Leg::calibrateFSRs(){
  foot_fsrs->calibrate();
}

void Leg::startTorqueCalibration(){
  for(int i = 0; i < motor_count;i++){
    motors[i]->startTorqueCalibration();
  }
}

void Leg::updateTorqueCalibration(){
  for(int i = 0; i < motor_count;i++){
    motors[i]->updateTorqueCalibration();
  }
}

void Leg::endTorqueCalibration(){
  for(int i = 0; i < motor_count;i++){
    motors[i]->endTorqueCalibration();
  }
}

bool Leg::checkMotorErrors(){
  for(int i = 0; i < motor_count;i++){
    if(motors[i]->hasErrored()){
      return true;
    }
  }
  return false;
}

void Leg::autoKF(){
  for(int i = 0; i < motor_count;i++){
    motors[i]->autoKF(state);
  }
}

void Leg::adjustControl(){
  for(int i = 0; i<motor_count;i++){
    motors[i]->adjustControl(state, state_old, FSR_baseline_FLAG);
  }
}

void Leg::resetStartingParameters(){
  this->activate_in_3_steps = 1;

  this->planter_step_count = 0;
  this->first_step = 1;

  for (int i = 0; i < motor_count; i++){
    motors[i]->resetStartingParameters();
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
  for(int i = 0; i < motor_count; i++){
    motors[i]->setTorqueScalar(activation_percent);
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

  for(int i = 0; i < motor_count; i++){
    motors[i]->changeState(state);
  }

  Phase new_phase = determinePhase(new_state, old_state, this->current_phase);
  if (new_phase != this->current_phase){
    this->current_phase = new_phase;
    this->changePhase(new_phase);
  }
}

void Leg::changePhase(int new_phase){
  if (new_phase == PLANTER){
    this->trigger_planter_phase();
  } else if (new_phase == DORSI){
    this->trigger_dorsi_phase();
  }
}

void Leg::applyControlAlgorithm(){
  if (this->current_phase == PLANTER){
    this->applyPlanterControlAlgorithm();
  } else if (this->current_phase == DORSI){
    this->applyDorsiControlAlgorithm();
  }
}

void Leg::applyPlanterControlAlgorithm(){
  for(int i =0; i < fsr_count; i++){
    fsrs[i]->updateMaxes();
  }

  double FSR_percentage = foot_fsrs->getPercentage();
  double max_FSR_percentage = foot_fsrs->getMaxPercentage();
  bool taking_baseline = false;

  for(int i = 0; i < motor_count; i++){
    motors[i]->applyPlanterControlAlgorithm(FSR_percentage, taking_baseline, max_FSR_percentage);
  }
}

void Leg::applyDorsiControlAlgorithm(){
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

void Leg::trigger_planter_phase(){
  this->planter_timer->reset();

  for (int i = 0; i <fsr_count;i++){
    fsrs[i]->trigger_planter_phase(dorsi_time);
  }

  for (int i = 0; i <motor_count;i++){
    motors[i]->trigger_planter_phase(dorsi_time);
  }
}

void Leg::trigger_dorsi_phase(){
  this->planter_step_count++; // you have accomplished a step
  this->dorsi_timer->reset();

  double planter_time = this->planter_timer->lap();
  this->planter_mean = this->planter_time_averager->update(planter_time);

  if (this->flag_N3_adjustment_time) {
    for (int i =0; i < motor_count; i++){
      motors[i]->adjustIterForTime();
    }
  }
}

int Leg::takeBaselineTriggerDorsiStart(){

  double planter_time = this->planter_timer->lap();
  this->planter_mean = this->planter_time_averager->update(planter_time);

  for(int i = 0; i < fsr_count;i++){
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
  for(int i = 0; i<motor_count; i++){
    motors[i]->setToZero();
  }
}

void Leg::applyStateMachine(){

  bool foot_on_ground = determine_foot_on_ground();
  int new_state = this->determineState(foot_on_ground);
  if (new_state != this->state){
    changeState(new_state, this->state);
  }
  updateIncrementalActivation();
  for(int i = 0; i < motor_count;i++){
    motors[i]->updateSetpoint(state);
  }
}

void Leg::measureSensors(){
  for(int i = 0; i < motor_count; i++){
    this->motors[i]->measureTorque();
    this->motors[i]->measureError();
  }
  this->foot_fsrs->measureForce();
}

void Leg::takeFSRBaseline(){
  if (FSR_baseline_FLAG){
    for (int i = 0; i < motor_count; i++){
      motors[i]->takeBaseline(state, state_old, &FSR_baseline_FLAG);
    }
  }
}

bool Leg::applyTorque(){
  for(int i = 0; i < motor_count; i++){
    if (!motors[i]->applyTorque(state)){
      state = 9;
      return false;
    }
  }
  return true;
}
