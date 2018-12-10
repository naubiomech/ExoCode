#include "Message.hpp"
#include "Leg.hpp"
#include "Joint.hpp"

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

LegMessage::LegMessage(LinkedList<Command<Leg>*>* pre_commands,
                       LinkedList<Command<Leg>*>* post_commands):Message<Leg>(pre_commands, post_commands){};

ExoMessage::ExoMessage(LinkedList<Command<Exoskeleton>*>* pre_commands,
                       LinkedList<Command<Exoskeleton>*>* post_commands):Message<Exoskeleton>(pre_commands, post_commands){};
