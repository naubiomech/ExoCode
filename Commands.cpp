#include "Commands.hpp"
#include "Exoskeleton.hpp"

template <class T>
Command<T>::~Command(){}

void StartTrialCommand::execute(Exoskeleton* exo){
  exo->startTrial();
}

void EndTrialCommand::execute(Exoskeleton* exo){
  exo->endTrial();
}

void CalibrateAllTorquesCommand::execute(Exoskeleton* exo){
  exo->calibrateTorque();
}

void CalibrateAllFsrsCommand::execute(Exoskeleton* exo){
  exo->calibrateFSRs();
}
