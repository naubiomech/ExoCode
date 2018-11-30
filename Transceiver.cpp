#include "Transceiver.hpp"
#include "Arduino.hpp"

Transceiver::Transceiver(TxPort* tx, RxPort* rx){
  command_serial = new SoftwareSerial(tx->getPin(), rx->getPin());
  command_serial->begin(115200);
  delete tx;
  delete rx;
}

Transceiver::~Transceiver(){
  /* delete command_serial; */
}

bool Transceiver::dataAvailable(){
  return command_serial->available() > 0;
}

bool Transceiver::noDataAvailable(){
  return !dataAvailable();
}

void Transceiver::receiveData(void* output_data_raw_form, int bytes_expected){
  char* output_data = (char*) output_data_raw_form;
  for(int i = 0; i < bytes_expected; i ++){
    if (noDataAvailable()){
      break;
    }
    int data = command_serial->read();
    output_data[i] = data;
  }
}

void Transceiver::sendCommandCode(CommandCode code){
  command_serial->write(code);
  command_serial->write(',');
}

void Transceiver::sendData(void* raw_data, int bytes_to_send){
  char* data = (char*) raw_data;
  for(int i = 0; i < bytes_to_send; i++){
    command_serial->write(data);
    command_serial->write(',');
  }
}

void Transceiver::sendCommandMessage(CommandCode command_code, void* data, int bytes_to_send) {
  sendMessageBegin();
  sendCommandCode(command_code);
  sendData(data, bytes_to_send);
  sendMessageEnd();
}

MatlabTransceiver::MatlabTransceiver(TxPort* tx, RxPort* rx):Transceiver(tx, rx){}

void MatlabTransceiver::receiveData(void* data, int doubles_expected){
  int new_data_size = doubles_expected * sizeof(double);
  Transceiver::receiveData(data, new_data_size);
}

void MatlabTransceiver::sendMessageBegin(){
  command_serial->print('S');
}

void MatlabTransceiver::sendMessageEnd(){
  command_serial->print('Z');
}

void MatlabTransceiver::sendData(void* raw_data, int doubles_to_send){
  double* data = (double*) raw_data;
  for (int i = 0; i < doubles_to_send; i++){
    command_serial->print(data[i]);
    command_serial->print(',');
  }
}
void Transceiver::sendLegReport(LegReport* report){
  double leg_data[5];
  leg_data[0] = report->joint_reports[0]->torque_sensor_report->measuredTorque;
  leg_data[1] = report->state;
  leg_data[2] = report->joint_reports[0]->pid_setpoint;
  leg_data[3] = report->fsr_reports[0]->threshold;
  leg_data[4] = report->fsr_reports[0]->measuredForce;
  sendData(leg_data, 5);
}

void Transceiver::sendReport(ExoReport* report){

  sendMessageBegin();
  sendLegReport(report->right_leg);
  sendLegReport(report->left_leg);

  double sigs[4];
  sigs[0] = 0; //SIG1
  sigs[1] = 0; //SIG2
  sigs[2] = 0; //SIG3
  sigs[3] = 0; //SIG4

  sendData(sigs, 4);
  sendMessageEnd();
}

ExoMessage* Transceiver::receiveMessages(ExoReport* report){
  ExoMessage* exoMsg = new ExoMessage();
  while(dataAvailable()){
    receiveMessage(exoMsg, report);
  }
  return exoMsg;
}

void Transceiver::receiveMessage(ExoMessage* exoMsg, ExoReport* report){

  if (noDataAvailable()) {
    return;
  }

  CommandCode cmd_from_Gui = command_serial->read();
  switch (cmd_from_Gui)
  {
  case COMM_CODE_REQUEST_DATA:
    sendReport(report);
    break;

  case COMM_CODE_START_TRIAL:
    /* exo->startTrial(); */
    break;

  case COMM_CODE_END_TRIAL:
    /* exo->endTrial(); */
    sendReport(report);
    break;

  case COMM_CODE_CALIBRATE_TORQUE:
    /* exo->calibrateTorque(); */
    break;

  case COMM_CODE_CHECK_BLUETOOTH:
    data_to_send[0] = 0;
    data_to_send[1] = 1;
    data_to_send[2] = 2;
    sendCommandMessage(COMM_CODE_CHECK_BLUETOOTH, data_to_send, 3);
    break;

  case COMM_CODE_CLEAN_BLUETOOTH_BUFFER:
    while (command_serial->available() > 0) command_serial->read();
    break;

  case COMM_CODE_GET_LEFT_ANKLE_SETPOINT:
    data_to_send[0] = report->left_leg->joint_reports[0]->pid_setpoint;
    sendCommandMessage(COMM_CODE_GET_LEFT_ANKLE_SETPOINT, data_to_send, 1);
    break;

  case COMM_CODE_GET_RIGHT_ANKLE_SETPOINT:
    data_to_send[0] = report->right_leg->joint_reports[0]->pid_setpoint;
    sendCommandMessage(COMM_CODE_GET_RIGHT_ANKLE_SETPOINT, data_to_send, 1);
    break;

  case COMM_CODE_SET_LEFT_ANKLE_SETPOINT:
    receiveData(data_received, 2);
    exoMsg->left_leg = prepareMotorMessage(report->left_leg, 0);
    exoMsg->left_leg->joint_messages[0]->motor_message->setpoint = data_received;
    /* exo->resetStartingParameters(); */
    break;

  case COMM_CODE_SET_RIGHT_ANKLE_SETPOINT:
    receiveData(data_received, 2);
    exoMsg->right_leg = prepareMotorMessage(report->right_leg, 0);
    exoMsg->right_leg->joint_messages[0]->motor_message->setpoint = data_received;
    /* exo->resetStartingParameters(); */
    break;
  }
  delete exoMsg;
}
