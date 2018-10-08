#include "Pins.h"


ExoPins::ExoPins(int motors_per_leg, int fsrs_per_leg){
  left_leg = new LegPins(motors_per_leg, fsrs_per_leg);
  right_leg = new LegPins(motors_per_leg, fsrs_per_leg);
}

ExoPins::~ExoPins(){
  delete left_leg;
  delete right_leg;
}

LegPins::~LegPins(){
  delete[] motor_pins;
  delete[] fsrs_pins;
}

LegPins::LegPins(int motor_count, int fsr_count){
  this->motor_pins = new MotorPins[motor_count];
  this->fsr_pins = new FSRPins[fsr_count];
  this->motor_count = motor_count;
  this->fsr_count = fsr_count;
}
