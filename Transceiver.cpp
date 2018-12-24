#include "Transceiver.hpp"
#include "Arduino.hpp"
#include "Exoskeleton.hpp"
#include "States.hpp"

bool Transceiver::noDataAvailable(){
  return !dataAvailable();
}

MatlabTransceiver::MatlabTransceiver(TxPort* tx, RxPort* rx){
  serial = new SoftwareSerial(tx->getPin(), rx->getPin());
  serial->begin(115200);
  delete tx;
  delete rx;
}

MatlabTransceiver::~MatlabTransceiver(){
  // Commented command serial delete until software serial allows deletion
  /* delete command_serial; */
}

bool MatlabTransceiver::dataAvailable(){
  return serial->available() > 0;
}

bool MatlabTransceiver::receiveHeader(){
  return serial->read() == 'S';
}

CommandCode MatlabTransceiver::receiveCommand(){
  return serial->read();
}

bool MatlabTransceiver::receiveFooter(){
  return true;
}

void MatlabTransceiver::receiveData(double* output_data, int doubles_expected){
  char* output_data_raw_form = (char*) output_data;
  for(int i = 0; i < doubles_expected; i ++){
    while (noDataAvailable()){}
    int data = serial->read();
    output_data_raw_form[i] = data;
  }
}

void MatlabTransceiver::sendHeader(){
  serial->write('S');
}

void MatlabTransceiver::sendCommand(CommandCode code){
  serial->write(code);
  serial->write(',');
}

void MatlabTransceiver::sendData(double* data, int doubles_to_send){
  for (int i = 0; i < doubles_to_send; i++){
    serial->print(data[i]);
    serial->write(',');
  }
}

void MatlabTransceiver::sendFooter(){
  serial->println('Z');
}
