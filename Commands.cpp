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

Command<Joint>* SetJointSetpointCommand::setParams(StateID state, double setpoint){
  this->state = state;
  this->setpoint = setpoint;
  return this;
}

void SetJointPidCommand::execute(Joint* joint){
  joint->setPid(p,i,d);
}

Command<Joint>* SetJointPidCommand::setParams(double p, double i, double d){
  this->p = p;
  this->i = i;
  this->d = d;
  return this;
}

void SetJointKfCommand::execute(Joint* joint){
  joint->setKf(kf);
}

Command<Joint>* SetJointKfCommand::setParams(double kf){
  this->kf = kf;
  return this;
}

void SetJointSmoothingParamCommand::execute(Joint* joint){
  joint->setSmoothingParam(state, param);
}

Command<Joint>* SetJointSmoothingParamCommand::setParams(StateID state, double param){
  this->state = state;
  this->param = param;
  return this;
}

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

Command<Joint>* ConcreteCommandFactory::createJointCommand(JointCommandID id){
  switch(id){
  case joint_ids.SET_SETPOINT:
    return new SetJointSetpointCommand();
  case joint_ids.SET_PID:
    return new SetJointPidCommand();
  case joint_ids.SET_KF:
    return new SetJointKfCommand();
  case joint_ids.SET_SMOOTHING:
    return new SetJointSetpointCommand();
  }
  return NULL;
}
