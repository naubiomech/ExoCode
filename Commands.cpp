#include "Commands.hpp"
#include "Exoskeleton.hpp"

void StartTrialCommand::execute(Exoskeleton* exo){
  exo->startTrial();
}
