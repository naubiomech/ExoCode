#include "Phase.hpp"
#include "Leg.hpp"

void Phase::triggerStart(){
  phase_time->reset();
}

void Phase::triggerEnd(){
  double time = phase_time->lap();
  phase_time_average->update(time);
}

double Phase::getAverageTime(){
  return phase_time_average->getAverage();
}

double Phase::getPhaseTime(){
  return phase_time->lap();
}

void Phase::setNextPhase(Phase* phase){
  next_phase = phase;
}

Phase* Phase::changePhase(){
  this->triggerEnd();
  next_phase->triggerStart();
  return next_phase;
}

void Phase::setContext(Leg* leg){
  this->leg = leg;
}

void DorsiPhase::run(){
  leg->runAutoKF();
}

void PlanterPhase::triggerStart(){
  Phase::triggerStart();
  leg->resetFSRMaxes();
  leg->incrementStepCount();
}

void PlanterPhase::run(){
  leg->updateFSRMaxes();
  leg->adjustSetpoint();
}

void PlanterPhase::triggerEnd(){
  Phase::triggerEnd();
  leg->adjustShapingForTime(getPhaseTime());
}
