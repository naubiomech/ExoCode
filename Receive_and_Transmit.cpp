#include <Arduino.h>
#include "Receive_and_Transmit.hpp"
#include <SoftwareSerial.h>
#include "Message.hpp"

// Peek is the variable used to identify the message received by matlab
// To understand the commands see the file .......... in the folder

void receive_data(SoftwareSerial* commandSerial, void* outputDataRawForm, int bytesExpected) {
  char* outputData = (char*) outputDataRawForm;
  int k = 0;
  while ((k < bytesExpected))
  {
    while (commandSerial->available() > 0)
    {
      int data = commandSerial->read();
      outputData[k] = data;               //Store all recieved bytes in subsequent memory spaces
      k++;                                  //Increments memory space
    }
  }
}

void send_leg_report(SoftwareSerial* commandSerial, LegReport* report){

  commandSerial->print(report->joint_reports[0]->torque_sensor_report->measuredTorque);
  commandSerial->print(',');
  commandSerial->print(report->state);
  commandSerial->print(',');
  commandSerial->print(report->joint_reports[0]->pid_setpoint);
  commandSerial->print(',');
  commandSerial->print(report->fsr_reports[0]->threshold);
  commandSerial->print(',');
  commandSerial->print(report->fsr_reports[0]->measuredForce);
  commandSerial->print(',');
}

void send_report(Exoskeleton* exo) {
  ExoReport* report = exo->report;
  exo->fillReport(report);
  SoftwareSerial* commandSerial = exo->commandSerial;

  commandSerial->print('S');
  commandSerial->print(',');
  send_leg_report(commandSerial, report->right_leg);
  send_leg_report(commandSerial, report->left_leg);

  commandSerial->print((double) (0)); //SIG1
  commandSerial->print(',');
  commandSerial->print((double) (0)); //SIG2
  commandSerial->print(',');
  commandSerial->print((double) (0)); //SIG3
  commandSerial->print(',');
  commandSerial->print((double) (0)); //SIG4

  commandSerial->print(',');
  commandSerial->println('Z');
}

void send_command_message(SoftwareSerial* commandSerial, char command_char, double* data_point, int bytes_to_send)
{
  int number_to_send = bytes_to_send / sizeof(*data_point);
  commandSerial->print('S');
  commandSerial->print(command_char);
  commandSerial->print(',');
  for (int message_iterator = 0; message_iterator < number_to_send; message_iterator++)
  {
    commandSerial->print(data_point[message_iterator]);
    commandSerial->print(',');
  }
  commandSerial->println('Z');
}

void receive_and_transmit(Exoskeleton* exo) {
  SoftwareSerial* commandSerial = exo->commandSerial;

  if (commandSerial->available() <= 0) {
    return;
  }

  ExoReport* report = exo->report;
  exo->fillReport(report);
  ExoMessage* exoMsg = new ExoMessage();
  double data_to_send[8];
  double data_received[8];

  int cmd_from_Gui = commandSerial->read();
  switch (cmd_from_Gui)
  {
  case COMM_CODE_REQUEST_DATA:
    send_report(exo);
    break;

  case COMM_CODE_START_TRIAL:
    exo->startTrial();
    break;

  case COMM_CODE_END_TRIAL:
    exo->endTrial();
    send_report(exo);
    break;

  case COMM_CODE_CALIBRATE_TORQUE:
    exo->calibrateTorque();
    break;

  case COMM_CODE_CHECK_BLUETOOTH:
    data_to_send[0] = 0;
    data_to_send[1] = 1;
    data_to_send[2] = 2;
    send_command_message(commandSerial, COMM_CODE_CHECK_BLUETOOTH, data_to_send, 3 * sizeof(*data_to_send));
    break;

  case COMM_CODE_CLEAN_BLUETOOTH_BUFFER:
    while (commandSerial->available() > 0) commandSerial->read();
    break;

  case COMM_CODE_GET_LEFT_ANKLE_SETPOINT:
    data_to_send[0] = report->left_leg->joint_reports[0]->pid_setpoint;
    send_command_message(commandSerial, COMM_CODE_GET_LEFT_ANKLE_SETPOINT, data_to_send, 1 * sizeof(*data_to_send));
    break;

  case COMM_CODE_GET_RIGHT_ANKLE_SETPOINT:
    data_to_send[0] = report->right_leg->joint_reports[0]->pid_setpoint;
    send_command_message(commandSerial, COMM_CODE_GET_RIGHT_ANKLE_SETPOINT, data_to_send, 1 * sizeof(*data_to_send));
    break;

  case COMM_CODE_SET_LEFT_ANKLE_SETPOINT:
    receive_data(commandSerial, data_received, 2 * sizeof(*data_received));
    exoMsg->left_leg = prepareMotorMessage(report->left_leg, 0);
    exoMsg->left_leg->joint_messages[0]->motor_message->setpoint = data_received;
    exo->resetStartingParameters();
    break;

  case COMM_CODE_SET_RIGHT_ANKLE_SETPOINT:
    receive_data(commandSerial, data_received, 2 * sizeof(*data_received));
    exoMsg->right_leg = prepareMotorMessage(report->right_leg, 0);
    exoMsg->right_leg->joint_messages[0]->motor_message->setpoint = data_received;
    exo->resetStartingParameters();
    break;
  }
  delete exoMsg;
}
