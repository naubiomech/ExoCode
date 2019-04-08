#include "Transceiver.hpp"
#include "Arduino.hpp"
#include "Exoskeleton.hpp"
#include "States.hpp"



Transceiver::~Transceiver(){
}

void MatlabTransceiver::clear(){
  while(dataAvailable()){
    serial->read();
  }
}

bool Transceiver::noDataAvailable(){
  return !dataAvailable();
}

MatlabTransceiver::MatlabTransceiver(TxPort* tx, RxPort* rx){
  serial = new SoftwareSerial(tx->getPin(), rx->getPin());
  serial->begin(115200);
  delete tx;
  delete rx;
}

MatlabTransceiver::MatlabTransceiver(SoftwareSerial* serial){
  this->serial = serial;
  serial->begin(115200);
}

MatlabTransceiver::~MatlabTransceiver(){
  // Commented command serial delete until software serial allows deletion
  /* delete command_serial; */
}

bool MatlabTransceiver::dataAvailable(){
  return serial->available() > 0;
}

bool MatlabTransceiver::receiveHeader(){
  return true;
}

CommandCode MatlabTransceiver::receiveCommand(){
  return serial->read();
}

bool MatlabTransceiver::receiveFooter(){
  return true;
}

void MatlabTransceiver::receiveData(double* output_data, unsigned int doubles_expected){
  if (doubles_expected <= 0){
    return;
  }

  unsigned int bytes_expected = doubles_expected * sizeof(double);

  char* output_data_raw_form = new char[bytes_expected];
  for(unsigned int i = 0; i < bytes_expected; i ++){
    while (noDataAvailable()){}
    int data = serial->read();
    output_data_raw_form[i] = data;
  }

  byte_transcriber.decodeDoubles(output_data, output_data_raw_form, doubles_expected);
  delete[] output_data_raw_form;
}

void MatlabTransceiver::sendHeader(){
  serial->write('S');
}

void MatlabTransceiver::sendCommand(CommandCode code){
  serial->write(code);
}

void MatlabTransceiver::sendData(float* data, unsigned int doubles_to_send){
	if (doubles_to_send <= 0){
		return;
	}
	serial->write((char) doubles_to_send);
	char* bytes = new char[doubles_to_send * sizeof(float)];

	byte_transcriber.encodeFloat(bytes, data, doubles_to_send);

	for (unsigned int i = 0; i < doubles_to_send * sizeof(float); i++){
		unsigned int d = bytes[i];
    serial->write(d);
  }

  delete[] bytes;
}

void MatlabTransceiver::sendFooter(){
}
