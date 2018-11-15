#include "Message.hpp"

JointMessage::~JointMessage(){
  delete motor_message;
  delete torque_sensor_message;
}

LegMessage::LegMessage(LegReport* report){
  fsr_message_count = report->fsr_report_count;
  joint_message_count = report->joint_report_count;
  imu_message_count = report->imu_report_count;

  fsr_messages = new FSRMessage*[fsr_message_count];
  joint_messages = new JointMessage*[joint_message_count];
  imu_messages = new IMUMessage*[imu_message_count];

  for (int i = 0; i < fsr_message_count; i++){
    fsr_messages[i] = NULL;
  }

  for (int i = 0; i < joint_message_count; i++){
    joint_messages[i] = NULL;
  }

  for (int i = 0; i < imu_message_count; i++){
    imu_messages[i] = NULL;
  }
}

LegMessage::~LegMessage(){

  for (int i = 0; i < fsr_message_count; i++){
    delete fsr_messages[i];
  }

  for (int i = 0; i < joint_message_count; i++){
    delete joint_messages[i];
  }

  for (int i = 0; i < imu_message_count; i++){
    delete imu_messages[i];
  }

  delete[] fsr_messages;
  delete[] joint_messages;
  delete[] imu_messages;
}

ExoMessage::~ExoMessage(){
  delete left_leg;
  delete right_leg;
}

LegMessage* prepareMotorMessage(LegReport* report, int joint_id){
  LegMessage* legMsg = new LegMessage(report);
  legMsg->joint_messages[joint_id] = new JointMessage();
  legMsg->joint_messages[joint_id]->motor_message = new MotorMessage();
  return legMsg;
}

LegMessage* prepareTorqueSensorMessage(LegReport* report, int joint_id){
  LegMessage* legMsg = new LegMessage(report);
  legMsg->joint_messages[joint_id] = new JointMessage();
  legMsg->joint_messages[joint_id]->torque_sensor_message = new TorqueSensorMessage();
  return legMsg;
}

LegMessage* prepareFSRMessage(LegReport* report, int fsr_id){
  LegMessage* legMsg = new LegMessage(report);
  legMsg->fsr_messages[fsr_id] = new FSRMessage();
  return legMsg;
}

LegMessage* prepareIMUMessage(LegReport* report, int imu_id){
  LegMessage* legMsg = new LegMessage(report);
  legMsg->imu_messages[imu_id] = new IMUMessage();
  return legMsg;
}
