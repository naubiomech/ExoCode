#ifndef EXO_MESSAGE_HEADER
#define EXO_MESSAGE_HEADER
#include <cstddef>
#include "Report.hpp"

enum LegMessageIndex{RIGHT_LEG_MSG, LEFT_LEG_MSG};

class Command{
public:
  virtual void execute();
};

class JointCommand:Command{
public:
  virtual void execute();
};

class LegCommand:Command{
public:
  virtual void execute();
};

class ExoCommand:Command{
public:
  virtual void execute();
};

class StartTrialCommand:ExoCommand{
public:
  virtual void execute();
};

class JointMessage{
public:
  JointCommand* command;
};

class LegMessage{
public:
  LegCommand* command;
  JointMessage* joint_msg;
  int joint_index;
};

class ExoMessage{
public:
  LegMessage* right_leg_msg;
  LegMessage* left_leg_msg;
  ExoCommand* command;
};

class MessageFactory{
  ExoMessage* createJointMessage(JointCommand* command, LegMessageIndex leg_index,int joint_index);
  ExoMessage* createLegMessage(JointCommand* command);
};
#endif
