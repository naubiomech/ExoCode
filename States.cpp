#include "States.hpp"
#include "Leg.hpp"

void State::triggerStart(){
  state_time->reset();
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
  leg->changeJointControl(next_state->getStateID());
  return next_state;
}

void State::setContext(Leg* leg){
  this->leg = leg;
}

void SwingState::run(){
  leg->runAutoKF();
}

StateID SwingState::getStateID(){
  return SWING;
}

void LateStanceState::triggerStart(){
  State::triggerStart();
  leg->resetFSRMaxes();
  leg->incrementStepCount();
  leg->setZeroIfNecessary();
}

void LateStanceState::triggerEnd(){
  State::triggerEnd();
  leg->adjustShapingForTime(getStateTime());
}

StateID LateStanceState::getStateID(){
  return LATE_STANCE;
}
