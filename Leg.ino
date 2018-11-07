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
  this->N3 = N3;
  this->N2 = N2;
  this->N1 = N1;

  this->activate_in_3_steps = 1;

  this->num_3_steps = 0;

  this->count_plant = 0;
  this->flag_N3_adjustment_time = false;
  this->first_step = 1;

  for (int i = 0; i < motor_count; i++){
    motors[i]->resetStartingParameters();
  }
}

void Leg::setZeroIfSteadyState(){
  if (this->flag_1 == 0) {
    this->flag_1 = 1;
    this->time_old_state = this->state;
  }
  if (this->state != this->time_old_state) {
    this->flag_1 = 0;
    this->stateTimerCount = 0;
  } else {
    if (this->stateTimerCount >= 5 / 0.002) {
      if (this->store_N1 == 0) {
        Serial.println("Steady state, setting to 0Nm , Change N1");
        this->set_2_zero = 1;
        this->store_N1 = 1;
        this->activate_in_3_steps = 1;
        this->num_3_steps = 0;
        this->first_step = 1;
        this->start_step = 0;
      }
    } else {
      this->stateTimerCount++;
      if (this->store_N1) {
        this->set_2_zero = 0;
        this->store_N1 = 0;
      }
    }
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

  motors[0]->applyPlanterControlAlgorithm(FSR_percentage, taking_baseline, max_FSR_percentage, control_algorithm);
  for(int i = 1; i < motor_count; i++){
    motors[i]->applyPlanterControlAlgorithm(FSR_percentage, taking_baseline, max_FSR_percentage, 0);
  }
}

void Leg::applyDorsiControlAlgorithm(){
  if (taking_baseline) {
    *p_Setpoint_Ankle_Pctrl = 0;
    *p_Setpoint_Ankle = 0;
  }
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

void Leg::applyStateMachine(){

  if (this->set_2_zero){
    switch (this->state) {
    case SWING:
      this->set_2_zero = 0;
      this->One_time_set_2_zero = 1;
      break;
    case LATE_STANCE:
      if (this->One_time_set_2_zero) {
        this->One_time_set_2_zero = 0;
        this->state_old = this->state;
        for(int i = 0; i<motor_count; i++){
          motors[i]->setToZero();
        }
      }
      break;
    }
  }

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
