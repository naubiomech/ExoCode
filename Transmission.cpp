#include "Transmission.hpp"
#include "Transceiver.hpp"
#include "JointSelect.hpp"
#include <string.h>

Transmission::Transmission(Transceiver* transceiver, CommandCode code,
                           unsigned int receive_count, unsigned int send_count){
  this->code = code;
  this->transceiver = transceiver;
  this->send_count = send_count;
  this->receive_count = receive_count;

  if (send_count > 0){
    send_data = new float[send_count];
  } else {
    send_data = NULL;
  }

  if (receive_count > 0){
    receive_data = new double[receive_count];
  } else {
    receive_data = NULL;
  }
}

Transmission::~Transmission(){
  delete[] send_data;
  delete[] receive_data;
}

void Transmission::preprocessData(){}

void Transmission::process(ExoMessageBuilder* builder, ExoReport* report){
  getData();
  preprocessData();
  processData(builder, report);
  postprocessData();
  sendData();
}

void Transmission::postprocessData(){}

void Transmission::getData(){
  if (receive_count > 0){
    transceiver->receiveData(receive_data, receive_count);
    transceiver->receiveFooter();
  }
}

void Transmission::sendData(){
  if (send_count > 0){
    transceiver->sendHeader();
    transceiver->sendCommand(code);
    transceiver->sendData(send_data, send_count);
    transceiver->sendFooter();
  }
}

void Transmission::copyToSend(double* from){
  memcpy(send_data, from, sizeof(float) * send_count);
}

void Transmission::copyFromReceive(double* to){
  memcpy(to, receive_data, sizeof(double) * receive_count);
}

unsigned int getJointSelectDataCount(unsigned int count, bool include_select, unsigned int select_count){
  if (include_select){
    count += select_count;
  }
  return count;
}

JointSelectTransmission::JointSelectTransmission(Transceiver* transceiver, CommandCode code,
                                                 unsigned int receive_count, bool receive_select,
                                                 unsigned int send_count, bool send_select):
Transmission(transceiver, code, getJointSelectDataCount(receive_count, receive_select, select_count),
             getJointSelectDataCount(send_count, send_select, select_count)){

  this->receive_count = receive_count;
  this->receive_select = receive_select;
  this->send_count = send_count;
  this->send_select = send_select;
}

void JointSelectTransmission::preprocessData(){
  if (receive_select){
    selects[0] = (int) (receive_data[0] + 0.1);
    selects[1] = (int) (receive_data[1] + 0.1);
    selects[2] = (int) (receive_data[2] + 0.1);
    for (unsigned int i = 0; i < receive_count; i++){
      receive_data[i] = receive_data[i+select_count];
    }
  }
}

void JointSelectTransmission::decodeJointSelect(int* selects, double encoded_select){
  byte_transcriber.decodeJointSelect(selects, encoded_select);
}

float JointSelectTransmission::encodeJointSelect(int* selects){
  return byte_transcriber.encodeJointSelect(selects);
}

void JointSelectTransmission::postprocessData(){
  if (send_select){
    for (int i = send_count - 1; i >= 0; i--){
      send_data[i+select_count] = send_data[i];
    }
    send_data[0] = (float) selects[0];
    send_data[1] = (float) selects[1];
    send_data[2] = (float) selects[2];
  }
}


RequestDataTransmission::RequestDataTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_REQUEST_DATA, 0, 14){}
void RequestDataTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  send_data[0] = report->right_leg->joint_reports[0]->torque_sensor_report->measuredTorque;
  send_data[1] = report->right_leg->state;
  send_data[2] = report->right_leg->joint_reports[0]->pid_setpoint;
  send_data[3] = report->right_leg->sensor_reports->fsr_reports[0]->threshold;
  send_data[4] = report->right_leg->sensor_reports->fsr_reports[0]->measuredForce;

  send_data[5] = report->left_leg->joint_reports[0]->torque_sensor_report->measuredTorque;
  send_data[6] = report->left_leg->state;
  send_data[7] = report->left_leg->joint_reports[0]->pid_setpoint;
  send_data[8] = report->left_leg->sensor_reports->fsr_reports[0]->threshold;
  send_data[9] = report->left_leg->sensor_reports->fsr_reports[0]->measuredForce;

  send_data[10] = 0;
  send_data[11] = 0;
  send_data[12] = 0;
  send_data[13] = 0;
}

StartTrialTransmission::StartTrialTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_START_TRIAL, 0, 0){}
void StartTrialTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->addPreCommand(new StartTrialCommand());
}

EndTrialTransmission::EndTrialTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_END_TRIAL, 0, 0){}
void EndTrialTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->addPreCommand(new EndTrialCommand());
}

CalibrateTorqueTransmission::CalibrateTorqueTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_CALIBRATE_TORQUE, 0, 0){}
void CalibrateTorqueTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->addPreCommand(new CalibrateAllTorquesCommand());
}

CheckBluetoothTransmission::CheckBluetoothTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_CHECK_BLUETOOTH, 0, 3){}
void CheckBluetoothTransmission::processData(ExoMessageBuilder*, ExoReport*){
  send_data[0] = 0;
  send_data[1] = 1;
  send_data[2] = 2;
}

CleanBluetoothBufferTransmission::CleanBluetoothBufferTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_CLEAN_BLUETOOTH_BUFFER, 0, 0){}
void CleanBluetoothBufferTransmission::processData(ExoMessageBuilder*, ExoReport*){
  transceiver->clear();
}

GetSetpointTransmission::GetSetpointTransmission(Transceiver* trans): JointSelectTransmission(trans, COMM_CODE_GET_TORQUE_SETPOINT, 0,true, 1, true){}
void GetSetpointTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  send_data[0] = report->getAreaReport(selects[0])->getJointReport(selects[1])->pid_setpoint;
}

SetSetpointTransmission::SetSetpointTransmission(Transceiver* trans):JointSelectTransmission(trans, COMM_CODE_SET_TORQUE_SETPOINT, 2,true, 0, false){}
void SetSetpointTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->
    beginAreaMessage(selects[0])->
    beginJointMessage(selects[1])->
    addCommand(new SetJointSetpointCommand(LATE_STANCE, receive_data[0]))->
    addCommand(new SetJointSetpointCommand(SWING, receive_data[1]));
}

CalibrateFsrTransmission::CalibrateFsrTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_CALIBRATE_FSR, 0, 0){}
void CalibrateFsrTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->addPreCommand(new CalibrateAllFsrsCommand());
}

GetFsrThresholdTransmission::GetFsrThresholdTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_GET_FSR_THRESHOLD, 0, 1){}
void GetFsrThresholdTransmission::processData(ExoMessageBuilder*, ExoReport*){
  send_data[0] = 1;
}

GetKFTransmission::GetKFTransmission(Transceiver* trans):JointSelectTransmission(trans, COMM_CODE_GET_KF, 0, true, 1, true){}
void GetKFTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  send_data[0] = report->getAreaReport(selects[0])->getJointReport(selects[1])->pid_kf;
}

SetKFTransmission::SetKFTransmission(Transceiver* trans):JointSelectTransmission(trans, COMM_CODE_SET_KF, 1, true, 0, false){}
void SetKFTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->
    beginAreaMessage(selects[0])->
    beginJointMessage(selects[1])->
    addCommand(new SetJointKfCommand(receive_data[0]));
}

GetPidParamsTransmission::GetPidParamsTransmission(Transceiver* trans):JointSelectTransmission(trans, COMM_CODE_GET_PID_PARAMS, 0, true, 3, true){}
void GetPidParamsTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  copyToSend(report->getAreaReport(selects[0])->getJointReport(selects[1])->pid_params);
}

SetPidParamsTransmission::SetPidParamsTransmission(Transceiver* trans):JointSelectTransmission(trans, COMM_CODE_SET_PID_PARAMS, 3, true, 0, false){}
void SetPidParamsTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->
    beginAreaMessage(selects[0])->
    beginJointMessage(selects[1])->
    addCommand(new SetJointPidCommand(receive_data[0], receive_data[1], receive_data[2]));
}

GetSmoothingParamsTransmission::GetSmoothingParamsTransmission(Transceiver* trans):JointSelectTransmission(trans, COMM_CODE_GET_SMOOTHING_PARAMS,0, true, 3, true){}
void GetSmoothingParamsTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  copyToSend(report->getAreaReport(selects[0])->getJointReport(selects[1])->smoothing);
}

SetSmoothingParamsTransmission::SetSmoothingParamsTransmission(Transceiver* trans):JointSelectTransmission(trans, COMM_CODE_SET_SMOOTHING_PARAMS, 3, true, 0, false){}
void SetSmoothingParamsTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->
    beginAreaMessage(selects[0])->
    beginJointMessage(selects[1])->
    addCommand(new SetJointSmoothingParamCommand(selects[2], receive_data[1]));
}

CheckMemoryTransmission::CheckMemoryTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_CHECK_MEMORY, 0, 3){}
void CheckMemoryTransmission::processData(ExoMessageBuilder*, ExoReport*){
  send_data[0] = 2;
  send_data[1] = 2;
  send_data[2] = 2;
}

Transmission* TransmissionFactory::create(Transceiver* trans, CommandCode code){
  switch (code) {
  case COMM_CODE_REQUEST_DATA:
    return new RequestDataTransmission(trans);
  case COMM_CODE_START_TRIAL:
    return new StartTrialTransmission(trans);
  case COMM_CODE_END_TRIAL:
    return new EndTrialTransmission(trans);
  case COMM_CODE_CALIBRATE_TORQUE:
    return new CalibrateTorqueTransmission(trans);
  case COMM_CODE_CHECK_BLUETOOTH:
    return new CheckBluetoothTransmission(trans);
  case COMM_CODE_CLEAN_BLUETOOTH_BUFFER:
    return new CleanBluetoothBufferTransmission(trans);
  case COMM_CODE_GET_TORQUE_SETPOINT:
    return new GetSetpointTransmission(trans);
  case COMM_CODE_SET_TORQUE_SETPOINT:
    return new SetSetpointTransmission(trans);
  case COMM_CODE_CALIBRATE_FSR:
    return new CalibrateFsrTransmission(trans);
  case COMM_CODE_GET_FSR_THRESHOLD:
    return new GetFsrThresholdTransmission(trans);
  case COMM_CODE_GET_KF:
    return new GetKFTransmission(trans);
  case COMM_CODE_SET_KF:
    return new SetKFTransmission(trans);
  case COMM_CODE_GET_PID_PARAMS:
    return new GetPidParamsTransmission(trans);
  case COMM_CODE_SET_PID_PARAMS:
    return new SetPidParamsTransmission(trans);
  case COMM_CODE_GET_SMOOTHING_PARAMS:
    return new GetSmoothingParamsTransmission(trans);
  case COMM_CODE_SET_SMOOTHING_PARAMS:
    return new SetSmoothingParamsTransmission(trans);
  case COMM_CODE_CHECK_MEMORY:
    return new CheckMemoryTransmission(trans);
  default:
    Serial.print("Command code not implemented: ");
    Serial.println(code);
  }
  return NULL;
}
