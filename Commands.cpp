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

void SetJointSetpointCommand::execute(Joint* joint){
  joint->setDesiredSetpoint(state, setpoint);
}

void SetJointPidCommand::execute(Joint* joint){
  joint->setPid(p,i,d);
}

void SetJointKfCommand::execute(Joint* joint){
  joint->setKf(kf);
}

void SetJointSmoothingParamCommand::execute(Joint* joint){
  joint->setSmoothingParam(state, param);
}

CommandFactory::CommandFactory(){}
CommandFactory::~CommandFactory(){}

ConcreteCommandFactory::ConcreteCommandFactory():CommandFactory(){}
ConcreteCommandFactory::~ConcreteCommandFactory(){}
Command<Exoskeleton>* ConcreteCommandFactory::createExoCommand(ExoCommandID id){
  switch(id){
  case exo_ids.START_TRIAL:
    return new StartTrialCommand();
  case exo_ids.END_TRIAL:
    return new EndTrialCommand();
  case exo_ids.CALIBRATE_TORQUE:
    return new CalibrateAllTorquesCommand();
  case exo_ids.CALIBRATE_FSRS:
    return new CalibrateAllFsrsCommand();
  }
  return NULL;
}

Command<Joint>* ConcreteCommandFactory::createJointCommand(JointCommandID id, StateID state, double value){
  switch(id){
  case joint_ids.SET_SETPOINT:
    return new SetJointSetpointCommand(state, value);
  case joint_ids.SET_SMOOTHING:
    return new SetJointSmoothingParamCommand(state, value);
  default:
    return NULL;
  }
}

Command<Joint>* ConcreteCommandFactory::createJointCommand(JointCommandID id, double v1, double v2, double v3){
  switch(id){
  case joint_ids.SET_PID:
    return new SetJointPidCommand(v1, v2, v3);
  default:
    return NULL;
  }
}

Command<Joint>* ConcreteCommandFactory::createJointCommand(JointCommandID id, double value){

  switch(id){
  case joint_ids.SET_KF:
    return new SetJointKfCommand(value);
  default:
    return NULL;
  }
}
