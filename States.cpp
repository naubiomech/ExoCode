#include "States.hpp"
#include "Leg.hpp"

void State::triggerStart(){
  state_time->reset();
  leg->scaleSetpointDifference();
}

void State::triggerEnd(){
}

double State::getStateTime(){
  return state_time->lap();
}

void State::setNextState(State* state){
  next_state = state;
}

State* State::changeState(){
  this->triggerEnd();
  next_state->triggerStart();
  return next_state;
}

void State::setContext(Leg* leg){
  this->leg = leg;
}

void SwingState::run(){
  leg->runAutoKF();
}

void LateStanceState::triggerStart(){
  State::triggerStart();
  leg->resetFSRMaxes();
  leg->incrementStepCount();
  leg->setZeroIfNecessary();
}

void LateStanceState::run(){
  leg->updateFSRMaxes();
  leg->adjustSetpoint();
}

void LateStanceState::triggerEnd(){
  State::triggerEnd();
  leg->adjustShapingForTime(getStateTime());
}
