#include <Arduino.h>
#include "Receive_and_Transmit.hpp"
#include <SoftwareSerial.h>

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

	commandSerial->print(report->motor_reports[0].measuredTorque);
	commandSerial->print(',');
	commandSerial->print(report->state);
	commandSerial->print(',');
	commandSerial->print(report->motor_reports[0].pid_setpoint);
	commandSerial->print(',');
	commandSerial->print(report->fsr_reports[0].threshold);
	commandSerial->print(',');
	commandSerial->print(report->fsr_reports[0].measuredForce);
	commandSerial->print(',');
	}

void send_report(ExoSystem* exoSystem) {
	ExoReport* report = exoSystem->report;
	exoSystem->exo->fillReport(report);
	SoftwareSerial* commandSerial = exoSystem->commandSerial;

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

void send_command_message(SoftwareSerial* commandSerial, char command_char, double* data_point, int number_to_send)
{
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

void receive_and_transmit(ExoSystem* exoSystem) {
// ===== Msg Functions =====
	double data_to_send[8];
	double *data_to_send_point = &data_to_send[0];

	SoftwareSerial* commandSerial = exoSystem->commandSerial;
	int cmd_from_Gui = commandSerial->read();
	switch (cmd_from_Gui)
	{
	case COMM_CODE_REQUEST_DATA:
		send_report(exoSystem);
		break;

	case COMM_CODE_START_TRIAL:
		exoSystem->startTrial();
		break;

	case COMM_CODE_END_TRIAL:
		exoSystem->endTrial();
		send_report(exoSystem);
		break;

	case COMM_CODE_CALIBRATE_TORQUE:
		exoSystem->exo->calibrateTorque();
		break;

	case COMM_CODE_CHECK_BLUETOOTH:
		data_to_send_point[0] = 0;
		data_to_send_point[1] = 1;
		data_to_send_point[2] = 2;
		send_command_message(commandSerial, COMM_CODE_CHECK_BLUETOOTH, data_to_send_point, 3);   //For the Arduino to prove to MATLAB that it is behaving, it will send back the character B
		break;

	case COMM_CODE_CLEAN_BLUETOOTH_BUFFER:
		while (commandSerial->available() > 0) commandSerial->read();
		break;
	}
}
