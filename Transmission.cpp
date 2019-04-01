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
    send_data = new double[send_count];
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


void Transmission::process(ExoMessageBuilder* builder, ExoReport* report){
  getData();
  processData(builder, report);
  sendData();
}

void Transmission::decodeJointSelect(int* selects, double encoded_select){
  byte_transcriber.decodeJointSelect(selects, encoded_select);
}


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
  memcpy(send_data, from, sizeof(double) * send_count);
}

void Transmission::copyFromReceive(double* to){
  memcpy(to, receive_data, sizeof(double) * receive_count);
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

GetSetpointTransmission::GetSetpointTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_GET_SETPOINT, 1, 1){}
void GetSetpointTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  int selects[3];
  decodeJointSelect(selects, receive_data[0]);

  send_data[0] = report->getAreaReport(selects[0])->getJointReport(selects[1])->pid_setpoint;
}

SetSetpointTransmission::SetSetpointTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_SET_SETPOINT, 3, 0){}
void SetSetpointTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  int selects[3];
  decodeJointSelect(selects, receive_data[0]);
  builder->
    beginAreaMessage(selects[0])->
    beginJointMessage(selects[1])->
    addCommand(new SetJointSetpointCommand(LATE_STANCE, receive_data[1]))->
    addCommand(new SetJointSetpointCommand(SWING, receive_data[2]));
}

CalibrateFsrTransmission::CalibrateFsrTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_CALIBRATE_FSR, 0, 0){}
void CalibrateFsrTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  builder->addPreCommand(new CalibrateAllFsrsCommand());
}

GetFsrThresholdTransmission::GetFsrThresholdTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_GET_FSR_THRESHOLD, 0, 1){}
void GetFsrThresholdTransmission::processData(ExoMessageBuilder*, ExoReport*){
  send_data[0] = 1;
}

GetKFTransmission::GetKFTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_GET_KF, 1, 1){}
void GetKFTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  int selects[3];
  decodeJointSelect(selects, receive_data[0]);

  send_data[0] = report->getAreaReport(selects[0])->getJointReport(selects[1])->pid_kf;
}

SetKFTransmission::SetKFTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_SET_KF, 2, 0){}
void SetKFTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  int selects[3];
  decodeJointSelect(selects, receive_data[0]);
  builder->
    beginAreaMessage(selects[0])->
    beginJointMessage(selects[1])->
    addCommand(new SetJointKfCommand(receive_data[1]));
}

GetPidParamsTransmission::GetPidParamsTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_GET_PID_PARAMS, 1, 3){}
void GetPidParamsTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  int selects[3];
  decodeJointSelect(selects, receive_data[0]);

  copyToSend(report->getAreaReport(selects[0])->getJointReport(selects[1])->pid_params);
}

SetPidParamsTransmission::SetPidParamsTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_SET_PID_PARAMS, 4, 0){}
void SetPidParamsTransmission::processData(ExoMessageBuilder* builder, ExoReport*){
  int selects[3];
  decodeJointSelect(selects, receive_data[0]);

  builder->
    beginAreaMessage(selects[0])->
    beginJointMessage(selects[1])->
    addCommand(new SetJointPidCommand(receive_data[1], receive_data[2], receive_data[3]));
}

GetSmoothingParamsTransmission::GetSmoothingParamsTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_GET_SMOOTHING_PARAMS, 1, 3){}
void GetSmoothingParamsTransmission::processData(ExoMessageBuilder*, ExoReport* report){
  int selects[3];
  decodeJointSelect(selects, receive_data[0]);

  copyToSend(report->getAreaReport(selects[0])->getJointReport(selects[1])->smoothing);
}

SetSmoothingParamsTransmission::SetSmoothingParamsTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_SET_SMOOTHING_PARAMS, 3, 0){}
void SetSmoothingParamsTransmission::processData(ExoMessageBuilder* builder, ExoReport*){

  int selects[3];
  decodeJointSelect(selects, receive_data[0]);

  builder->
    beginAreaMessage(selects[0])->
    beginJointMessage(selects[1])->
    addCommand(new SetJointSmoothingParamCommand(selects[2], receive_data[1]));
}

CheckMemoryTransmission::CheckMemoryTransmission(Transceiver* trans):Transmission(trans, COMM_CODE_CHECK_MEMORY, 0, 3){}
void CheckMemoryTransmission::processData(ExoMessageBuilder*, ExoReport* report){
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
  case COMM_CODE_GET_SETPOINT:
    return new GetSetpointTransmission(trans);
  case COMM_CODE_SET_SETPOINT:
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
