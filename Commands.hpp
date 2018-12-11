#ifndef EXO_COMMAND_HEADER
#define EXO_COMMAND_HEADER
#include "States.hpp"

class Exoskeleton;
class Leg;
class Joint;

template <class T>
class Command{
public:
  virtual ~Command();
  virtual void execute(T* context) = 0;
};

class StartTrialCommand:public Command<Exoskeleton>{
public:
  virtual void execute(Exoskeleton* exo);
};

class EndTrialCommand:public Command<Exoskeleton>{
public:
  virtual void execute(Exoskeleton* exo);
};

class CalibrateAllTorquesCommand:public Command<Exoskeleton>{
public:
  virtual void execute(Exoskeleton* exo);
};

class CalibrateAllFsrsCommand:public Command<Exoskeleton>{
public:
  virtual void execute(Exoskeleton* exo);
};

class SetJointSetpoint:public Command<Joint>{
private:
  StateID state;
  double setpoint;
public:
SetJointSetpoint(StateID state, double setpoint): state(state), setpoint(setpoint){};
  virtual void execute(Joint* joint);
};

#endif
