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
  LegMessage(LinkedList<Command<Leg>*>* pre_commands, LinkedList<Command<Leg>*>* post_commands,
             LinkedList<JointMessage*>* joint_msgs);
  void messageJoints(LinkedList<Joint*>* joints);
};

class ExoMessage:public Message<Exoskeleton>{
private:
  LegMessage* right_leg_msg;
  LegMessage* left_leg_msg;
public:
  ExoMessage(LinkedList<Command<Exoskeleton>*>* pre_commands, LinkedList<Command<Exoskeleton>*>* post_commands,
             LegMessage* right_leg_msg, LegMessage* left_leg_msg);
  void messageRightLeg(Leg* right);
  void messageLeftLeg(Leg* left);
};



template <class T, class Context>
class MessageBuilder{
private:
  LinkedList<Command<T>*>  pre_commands;
  LinkedList<Command<T>*>  post_commands;
protected:
  LinkedList<Command<T>*>* getPreCommands();
  LinkedList<Command<T>*>* getPostCommands();
  void clearCommands();
public:
  Context* addPreCommand(Command<T>* command);
  Context* addPostCommand(Command<T>* command);
};

template <class T, class Context>
class SingleMessageBuilder:private MessageBuilder<T, Context>{
protected:
  LinkedList<Command<T>*>* getCommands();
  void clearCommands();
public:
  Context* addCommand(Command<T>* command);
};
class JointMessageBuilder;
class LegMessageBuilder;
class ExoMessageBuilder;


class JointMessageBuilder:public SingleMessageBuilder<Joint, JointMessageBuilder>{
private:
  LegMessageBuilder* return_context;
public:
  JointMessageBuilder(LegMessageBuilder* return_context);
  LegMessageBuilder* finishJoint();
  JointMessage* build();
};

class LegMessageBuilder:public MessageBuilder<Leg, LegMessageBuilder>{
private:
  ExoMessageBuilder* return_context;
  LinkedList<JointMessageBuilder*> joint_builders;
public:
  LegMessageBuilder(ExoMessageBuilder* return_context);
  JointMessageBuilder* beginJointMessage(unsigned int id);
  ExoMessageBuilder* finishLeg();
  LegMessage* build();
};

class ExoMessageBuilder:public MessageBuilder<Exoskeleton, ExoMessageBuilder>{
private:
  LegMessageBuilder* left_builder;
  LegMessageBuilder* right_builder;
public:
  ExoMessageBuilder();
  LegMessageBuilder* beginLeftLegMessage();
  LegMessageBuilder* beginRightLegMessage();
  ExoMessage* build();
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

template<class T,class Context>
Context* MessageBuilder<T, Context>::addPreCommand(Command<T>* command){
  pre_commands.append(command);
  return this;
}

template<class T, class Context>
Context* MessageBuilder<T, Context>::addPostCommand(Command<T>* command){
  post_commands.append(command);
  return this;
}

template<class T, class Context>
LinkedList<Command<T>*>* MessageBuilder<T, Context>::getPreCommands(){
  return &pre_commands;
}

template<class T, class Context>
LinkedList<Command<T>*>* MessageBuilder<T, Context>::getPostCommands(){
  return &post_commands;
}

template<class T, class Context>
void MessageBuilder<T, Context>::clearCommands(){
  pre_commands.clear();
  post_commands.clear();
}

template<class T, class Context>
Context* SingleMessageBuilder<T, Context>::addCommand(Command<T>* command){
  this->addPreCommand(command);
  return this;
}

template<class T, class Context>
LinkedList<Command<T>*>* SingleMessageBuilder<T, Context>::getCommands(){
  return this->getPreCommands();
}


template<class T, class Context>
void SingleMessageBuilder<T, Context>::clearCommands(){
  MessageBuilder<T, Context>::clearCommands();
}
#endif
