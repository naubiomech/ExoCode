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

  this->count_plant = 0;
  this->flag_N3_adjustment_time = false;
  this->first_step = 1;

  for (int i = 0; i < motor_count; i++){
    motors[i]->resetStartingParameters();
  }
}

void Leg::IsSteadyState(){
  return this->stateTime->lap() > STEADY_STATE_TIMEOUT;
}

void Leg::startIncrementalActivation(){
  this->increment_activation_starting_step = 0;
}

void Leg::updateIncrementalActivation(){
  double since_activation_step_count = this->step_count - this->increment_activation_starting_steps;
  double activation_percent = since_activation_step_count / activation_step_count;
  for(int i = 0; i < motor_count; i++){
    motors[i]->setTorqueScalar(completion_percent);
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

void Leg::changeState(int new_state){
  this->state_old = this->state;
  this->state = new_state;
  this->stateTime->reset();

  for(int i = 0; i < motor_count; i++){
    motors[i]->changeState(state);
  }

  int new_phase = determinePhase(this->state, this->old_state, this->phase);
  if (new_phase != this->phase){
    this->phase = new_phase;
    this->changePhase(new_phase);
  }
}

void Leg::changePhase(int new_phase){
  if (new_phase == PLANTER){
    this->trigger_planter_phase();
  } else if (next_phase == DORSI){
    this->trigger_dorsi_phase();
  }
}

void Leg::applyControlAlgorithm(){
  if (this->phase == PLANTER){
    this->applyPlanterControlAlgorithm();
  } else if (this->phase == DORSI){
    this->applyDorsiControlAlgorithm();
  }
}

void Leg::applyPlanterControlAlgorithm(){
  for(int i =0; i < fsr_count; i++){
    fsrs[i]->updateMaxes();
  }

  double FSR_percentage = foot_fsrs->getRatio();
  double max_FSR_percentage = foot_fsrs->getMaxRatio();

  for(int i = 0; i < motor_count; i++){
    motors[i]->applyPlanterControlAlgorithm(FSR_percentage, taking_baseline, max_FSR_percentage);
  }
}

void Leg::applyDorsiControlAlgorithm(){
}

int Leg::determinePhase(int new_state, int old_state, int current_phase){
  double current_phase_time;
  int next_phase;
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
  this->plant_timer->reset();

  for (int i = 0; i <fsr_count;i++){
    fsrs[i]->trigger_dorsi_phase(plant_time);
  }

  for (int i = 0; i <motor_count;i++){
    motors[i]->trigger_dorsi_phase(plant_time);
  }
}

void Leg::trigger_dorsi_phase(){
  this->count_plant++; // you have accomplished a step
  this->dorsi_timer->reset();

  double plant_time = this->plant_timer->lap();
  this->plant_mean = this->planter_time_average->update(plant_time);

  if (this->flag_N3_adjustment_time) {
    for (int i =0; i < motor_count; i++){
      motors[i]->adjustIterForTime();
    }
  }
}

int Leg::takeBaselineTriggerDorsiStart(){

  double plant_time = this->plant_timer->lap();
  this->plant_mean = this->plant_time_averager->updateAverage(plant_time);

  for(int i = 0; i < fsr_count;i++){
    fsrs[i]->updateBaseline();
  }

  if (this->count_plant_base >= n_step_baseline){
    this->count_plant_base = 0;

    return 0;
  }

  return 1;
}

int Leg::takeBaselineTriggerPlanterStart(){

  double dorsi_time = this->dorsi_timer->lap();
  this->dorsi_mean = this->dorsi_time_averager->updateAverage(dorsi_time);

  return 1;
}

void Leg::triggerLateStance(){
  if (this->set_motors_to_zero_torque){
    setToZero();
    this->set_motors_to_zero_torque = 0;
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
  this->determineState(foot_on_ground);
  if (new_state != this->state){
    changeState(new_state);
  }
  ref_step_adj(this);
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
