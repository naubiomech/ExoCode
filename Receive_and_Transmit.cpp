#include <Arduino.h>
#include "Receive_and_Transmit.hpp"

// Peek is the variable used to identify the message received by matlab
// To understand the commands see the file .......... in the folder

void receive_data(SoftwareSerial* commandSerial, void* outputDataRawForm, int bytesExpected)
{
  char* outputData = (char*) outputDataRawForm;
  int k = 0;
  while ((k < bytesExpected))
  {
    while (commandSerial->available() > 0)
    {
      int data = commandSerial->read();
      holdon[k] = data;               //Store all recieved bytes in subsequent memory spaces
      k++;                                  //Increments memory space
    }
  }
}


void send_report(SoftwareSerial* commandSerial) //with COP
{
  /*
    commandSerial->print('S');
    commandSerial->print(',');

    // RIGHT
    commandSerial->print(exo->getRightAverageTorque());
    commandSerial->print(',');
    commandSerial->print(exo->getRightLegState());
    commandSerial->print(',');
    commandSerial->print(right_leg->sign * right_leg->PID_Setpoint);
    commandSerial->print(',');
    if (FLAG_TWO_TOE_SENSORS) {
    commandSerial->print(right_leg->fsr_percent_thresh_Toe * right_leg->fsr_Combined_peak_ref);
    commandSerial->print(',');
    commandSerial->print(right_leg->FSR_Combined_Average);
    commandSerial->print(',');
    } else {
    commandSerial->print(right_leg->fsr_percent_thresh_Toe * right_leg->fsr_Toe_peak_ref);
    commandSerial->print(',');
    commandSerial->print(right_leg->FSR_Toe_Average);
    commandSerial->print(',');
    }

    // LEFT
    commandSerial->print(exo->getLeftAverageTorque());
    commandSerial->print(',');
    commandSerial->print(exo->getLeftLegState());
    commandSerial->print(',');
    commandSerial->print(left_leg->sign * left_leg->PID_Setpoint);
    commandSerial->print(',');
    if (FLAG_TWO_TOE_SENSORS) {
    commandSerial->print(left_leg->fsr_percent_thresh_Toe * left_leg->fsr_Combined_peak_ref);
    commandSerial->print(',');
    commandSerial->print(left_leg->FSR_Combined_Average);
    commandSerial->print(',');
    } else {
    commandSerial->print(left_leg->fsr_percent_thresh_Toe * left_leg->fsr_Toe_peak_ref);
    commandSerial->print(',');
    commandSerial->print(left_leg->FSR_Toe_Average);
    commandSerial->print(',');
    }


  commandSerial->print((left_leg->Vol)); //SIG1
  commandSerial->print(',');
  commandSerial->print((right_leg->Vol)); //SIG2
  commandSerial->print(',');
  commandSerial->print(left_leg->FSR_Ratio); //SIG3
  commandSerial->print(',');
  commandSerial->print(right_leg->FSR_Ratio); //SIG4

  commandSerial->print(',');
  commandSerial->println('Z');
  */
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

void receive_and_transmit(System* system)
{
// ===== Msg Functions =====
  double data_to_send[8];
  double *data_to_send_point = &data_to_send[0];

  SoftwareSerial* commandSerial = system->commandSerial;
  int cmd_from_Gui = commandSerial->read();
  switch (cmd_from_Gui)
  {
  case COMM_CODE_REQUEST_DATA:
    send_data_message_wc();
    break;

  case COMM_CODE_START_TRIAL:
    exo->enableExo();
    stream = 1;                                                     //and the torque data is allowed to be streamed
    streamTimerCount = 0;
    break;

  case COMM_CODE_END_TRIAL:
    exo->disableExo();
    stream = 0;                                                    //and the torque data is no longer allowed to be streamed.
    break;

  case COMM_CODE_CALIBRATE_TORQUE:
    exo->calibrateTorque();
    break;

  case COMM_CODE_CHECK_BLUETOOTH:
    data_to_send_point[0] = 0;
    data_to_send_point[1] = 1;
    data_to_send_point[2] = 2;
    send_command_message(COMM_CODE_CHECK_BLUETOOTH, data_to_send_point, 3);   //For the Arduino to prove to MATLAB that it is behaving, it will send back the character B
    break;

  case COMM_CODE_CLEAN_BLUETOOTH_BUFFER:
    while (bluetooth->available() > 0) bluetooth->read();
    break;
  }
}
