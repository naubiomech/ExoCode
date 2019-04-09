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
  virtual void execute(Joint* joint);
  virtual Command<Joint>* setParams(StateID state, double setpoint);
};

class SetJointPidCommand:public Command<Joint>{
private:
  double p;
  double i;
  double d;
public:
  virtual void execute(Joint* joint);
  virtual Command<Joint>* setParams(double p, double i, double d);
};

class SetJointKfCommand:public Command<Joint>{
private:
  double kf;
public:
  virtual void execute(Joint* joint);
  virtual Command<Joint>* setParams(double kf);
};

class SetJointSmoothingParamCommand:public Command<Joint>{
private:
  StateID state;
  double param;
public:
  virtual void execute(Joint* joint);
  virtual Command<Joint>* setParams(StateID state, double params);
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
  virtual Command<Joint>* createJointCommand(JointCommandID id) = 0;
  virtual Command<Exoskeleton>* createExoCommand(ExoCommandID) = 0;
};

class ConcreteCommandFactory{
private:
  ExoCommandIDs exo_ids;
  JointCommandIDs joint_ids;
public:
  virtual Command<Joint>* createJointCommand(JointCommandID id);
  virtual Command<Exoskeleton>* createExoCommand(ExoCommandID id);
};

#endif
