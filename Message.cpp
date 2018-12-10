#include "Message.hpp"
#include "Leg.hpp"
#include "Joint.hpp"

LegMessage::LegMessage(LinkedList<Command<Leg>*>* pre_commands,
                       LinkedList<Command<Leg>*>* post_commands,
                       LinkedList<JointMessage*>* joint_msgs):Message<Leg>(pre_commands, post_commands){
                         this->joint_msgs = joint_msgs;
};


void LegMessage::messageJoints(LinkedList<Joint*>* joints){
  ListIterator<Joint*> joint_iter = joints->getIterator();
  ListIterator<JointMessage*> msg_iter = joint_msgs->getIterator();
  while(joint_iter.hasNext() && msg_iter.hasNext()){
    joint_iter.next()->processMessage(msg_iter.next());
  }
}

void ExoMessage::messageLeftLeg(Leg* left){
  left->processMessage(left_leg_msg);
}

void ExoMessage::messageRightLeg(Leg* right){
  right->processMessage(right_leg_msg);
}

JointMessage::JointMessage(LinkedList<Command<Joint>*>* commands):Message<Joint>(commands){};

ExoMessage::ExoMessage(LinkedList<Command<Exoskeleton>*>* pre_commands,
                       LinkedList<Command<Exoskeleton>*>* post_commands,
                       LegMessage* right_leg_msg,
                       LegMessage* left_leg_msg):Message<Exoskeleton>(pre_commands, post_commands){
                         this->right_leg_msg = right_leg_msg;
                         this->left_leg_msg = left_leg_msg;
};

JointMessageBuilder::JointMessageBuilder(LegMessageBuilder* return_context){
  this->return_context = return_context;
}

LegMessageBuilder* JointMessageBuilder::finishJoint(){
  return return_context;
}

JointMessage* JointMessageBuilder::build(){
  JointMessage* joint_msg = new JointMessage(getCommands());
  clearCommands();
  return joint_msg;
}

LegMessageBuilder::LegMessageBuilder(ExoMessageBuilder* return_context){
  this->return_context = return_context;
}

JointMessageBuilder* LegMessageBuilder::beginJointMessage(unsigned int id){
  while ((id+1) < joint_builders.size()){
    joint_builders.append(new JointMessageBuilder(this));
  }
  return joint_builders[id];
}

ExoMessageBuilder* LegMessageBuilder::finishLeg(){
  return return_context;
}

LegMessage* LegMessageBuilder::build(){
  LinkedList<JointMessage*>* joint_msgs = new LinkedList<JointMessage*>();
  ListIterator<JointMessageBuilder*> joint_builder_iter = joint_builders.getIterator();
  while(joint_builder_iter.hasNext()){
    joint_msgs->append(joint_builder_iter.next()->build());
  }
  LegMessage* leg_msg = new LegMessage(getPreCommands(), getPostCommands(), joint_msgs);
  clearCommands();
  return leg_msg;
}

ExoMessageBuilder::ExoMessageBuilder(){
  left_builder = NULL;
  right_builder = NULL;
}

LegMessageBuilder* ExoMessageBuilder::beginLeftLegMessage(){
  if(left_builder == NULL){
    left_builder = new LegMessageBuilder(this);
  }
  return left_builder;
}

LegMessageBuilder* ExoMessageBuilder::beginRightLegMessage(){
  if(right_builder == NULL){
    right_builder = new LegMessageBuilder(this);
  }
  return right_builder;
}

ExoMessage* ExoMessageBuilder::build(){
  ExoMessage* exo_msg = new ExoMessage(getPreCommands(), getPostCommands(),
                                       right_builder->build(), left_builder->build());
  this->clearCommands();
  return exo_msg;
}
