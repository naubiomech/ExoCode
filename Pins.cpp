#include <Arduino.h>
#include "Pins.hpp"


ExoPins::ExoPins(int joints_per_leg, int fsrs_per_leg){
  left_leg = new LegPins(joints_per_leg, fsrs_per_leg);
  right_leg = new LegPins(joints_per_leg, fsrs_per_leg);
}

ExoPins::~ExoPins(){
  delete left_leg;
  delete right_leg;
}

LegPins::~LegPins(){
  delete[] joint_pins;
  delete[] fsr_pins;
}

LegPins::LegPins(int joint_count, int fsr_count){
  this->joint_pins = new JointPins[joint_count];
  this->fsr_pins = new FSRPins[fsr_count];
  this->joint_count = joint_count;
  this->fsr_count = fsr_count;
}
