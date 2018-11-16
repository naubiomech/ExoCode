#include <Arduino.h>
#include "Pins.hpp"
#include "Board.hpp"

ExoPins::ExoPins(int joints_per_leg, int fsr_groups_per_leg, int fsrs_per_group, int imus_per_leg){
  left_leg = new LegPins(joints_per_leg, fsr_groups_per_leg, fsrs_per_group, imus_per_leg, LEFT_LEG_SIGN);
  right_leg = new LegPins(joints_per_leg, fsr_groups_per_leg, fsrs_per_group, imus_per_leg, RIGHT_LEG_SIGN);
}

ExoPins::~ExoPins(){
  delete left_leg;
  delete right_leg;
}

LegPins::~LegPins(){
  for(int i = 0; i < alloced_joint_count; i++){
    delete joint_pins[i];
  }

  for(int i = 0; i < alloced_fsr_group_count; i++){
    delete fsr_groups_pins[i];
  }

  for(int i = 0; i < alloced_imu_count; i++){
    delete imu_pins[i];
  }

  delete[] joint_pins;
  delete[] fsr_groups_pins;
  delete[] imu_pins;
}

LegPins::LegPins(int joint_count, int fsr_group_count, int fsrs_per_group, int imu_count, int leg_sign){
  this->leg_sign = leg_sign;
  this->joint_pins = new JointPins*[joint_count];
  this->fsr_groups_pins = new FSRGroupPins*[fsr_group_count];
  this->imu_pins = new IMUPins*[imu_count];

  this->joint_count = joint_count;
  this->fsr_group_count = fsr_group_count;
  this->imu_count = imu_count;

  this->alloced_joint_count = joint_count;
  this->alloced_fsr_group_count = fsr_group_count;
  this->alloced_imu_count = imu_count;


  for(int i = 0; i < joint_count; i++){
    joint_pins[i] = new JointPins();
  }

  for(int i = 0; i < fsr_group_count; i++){
    fsr_groups_pins[i] = new FSRGroupPins(fsrs_per_group);
  }

  for(int i = 0; i < imu_count; i++){
    imu_pins[i] = new IMUPins();
  }
}

JointPins::JointPins(){
  motor_pins = new MotorPins();
  torque_sensor_pins = new TorqueSensorPins;
}

JointPins::~JointPins(){
  delete motor_pins;
  delete torque_sensor_pins;
}

FSRGroupPins::FSRGroupPins(int fsr_count){
  this->fsr_pins = new int[fsr_count];
  this->fsr_count = fsr_count;
}

FSRGroupPins::~FSRGroupPins(){
  delete[] fsr_pins;
}
