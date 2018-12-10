#ifndef EXO_MESSAGE_HEADER
#define EXO_MESSAGE_HEADER
#include <cstddef>
#include "Report.hpp"
#include "Linked_List.hpp"
#include "Commands.hpp"

class Exoskeleton;
class Leg;
class Joint;

template <class Context>
class Message{
private:
  LinkedList<Command<Context>*>* pre_commands;
  LinkedList<Command<Context>*>* post_commands;
  void runCommands(Context* context, LinkedList<Command<Context>*>* cmds);
public:
  Message(LinkedList<Command<Context>*>* commands);
  Message(LinkedList<Command<Context>*>* pre_commands, LinkedList<Command<Context>*>* post_commands);
  void runPreCommands(Context* context);
  void runCommands(Context* context);
  void runPostCommands(Context* context);
};

class JointMessage:public Message<Joint>{
public:
  JointMessage(LinkedList<Command<Joint>*>* commands);
};

class LegMessage:public Message<Leg>{
private:
  LinkedList<JointMessage*>* joint_msgs;
public:
  LegMessage(LinkedList<Command<Leg>*>* pre_commands, LinkedList<Command<Leg>*>* post_commands);
  void messageJoints(LinkedList<Joint*>* joints);
};

class ExoMessage:public Message<Exoskeleton>{
private:
  LegMessage* right_leg_msg;
  LegMessage* left_leg_msg;
public:
  ExoMessage(LinkedList<Command<Exoskeleton>*>* pre_commands, LinkedList<Command<Exoskeleton>*>* post_commands);
  void messageRightLeg(Leg* right);
  void messageLeftLeg(Leg* left);
};

template<class Context>
Message<Context>::Message(LinkedList<Command<Context>*>* commands){
  pre_commands = commands;
  post_commands = NULL;
}

template<class Context>
Message<Context>::Message(LinkedList<Command<Context>*>* pre_commands, LinkedList<Command<Context>*>* post_commands){
  this->pre_commands = pre_commands;
  this->post_commands = post_commands;
}

template<class Context>
void Message<Context>::runCommands(Context* context, LinkedList<Command<Context>*>* cmds){
  if (cmds == NULL){
    return;
  }

  ListIterator<Command<Context>*> iter = cmds->getIterator();
  while(iter.hasNext()){
    Command<Context>* cmd = iter.next();
    cmd->execute(context);
  }
}

template<class Context>
void Message<Context>::runPreCommands(Context* context){
  runCommands(context, pre_commands);
}

template<class Context>
void Message<Context>::runCommands(Context* context){
  runPreCommands(context);
}

template<class Context>
void Message<Context>::runPostCommands(Context* context){
  runCommands(context, post_commands);
}

#endif
