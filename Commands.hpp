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

class SetJointSetpointCommand:public Command<Joint>{
private:
  StateID state;
  double setpoint;
public:
SetJointSetpointCommand(StateID state, double setpoint): state(state), setpoint(setpoint){};
  virtual void execute(Joint* joint);
};

class SetJointPidCommand:public Command<Joint>{
private:
  double p;
  double i;
  double d;
public:
SetJointPidCommand(double p, double i, double d): p(p), i(i), d(d){};
  virtual void execute(Joint* joint);
};

class SetJointKfCommand:public Command<Joint>{
private:
  double kf;
public:
SetJointKfCommand(double kf): kf(kf){};
  virtual void execute(Joint* joint);
};

class SetJointSmoothingParamCommand:public Command<Joint>{
private:
  StateID state;
  double param;
public:
SetJointSmoothingParamCommand(StateID state, double param): state(state), param(param){};
  virtual void execute(Joint* joint);
};

typedef int ExoCommandID;
class ExoCommandIDs{
public:
  static const ExoCommandID START_TRIAL = 0;
  static const ExoCommandID END_TRIAL = 1;
  static const ExoCommandID CALIBRATE_TORQUE = 2;
  static const ExoCommandID CALIBRATE_FSRS = 3;
};

typedef int JointCommandID;
class JointCommandIDs{
public:
  static const JointCommandID SET_SETPOINT = 0;
  static const JointCommandID SET_PID = 1;
  static const JointCommandID SET_KF = 2;
  static const JointCommandID SET_SMOOTHING = 3;
};

class CommandFactory{
public:
  CommandFactory();
  virtual ~CommandFactory();
  virtual Command<Joint>* createJointCommand(JointCommandID id, StateID state, double value) = 0;
  virtual Command<Joint>* createJointCommand(JointCommandID id, double, double, double) = 0;
  virtual Command<Joint>* createJointCommand(JointCommandID id, double) = 0;
  virtual Command<Exoskeleton>* createExoCommand(ExoCommandID) = 0;
};

class ConcreteCommandFactory : public CommandFactory{
private:
  ExoCommandIDs exo_ids;
  JointCommandIDs joint_ids;
public:
  ConcreteCommandFactory();
  ~ConcreteCommandFactory();
  virtual Command<Exoskeleton>* createExoCommand(ExoCommandID id);
  virtual Command<Joint>* createJointCommand(JointCommandID id, StateID state, double value);
  virtual Command<Joint>* createJointCommand(JointCommandID id, double, double, double);
  virtual Command<Joint>* createJointCommand(JointCommandID id, double);
};

#endif
