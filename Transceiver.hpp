#ifndef TRANSCEIVER_HEADER
#define TRANSCEIVER_HEADER
#include "Arduino.hpp"
#include "Port.hpp"
#include "Command_Codes.hpp"
#include "Report.hpp"
#include "Message.hpp"

class Exoskeleton;
class Transceiver{
private:
  void receiveMessage(ExoMessage* msg, ExoReport* report);
  bool dataAvailable();
  bool noDataAvailable();
  void sendLegReport(LegReport* report);
protected:
  SoftwareSerial* command_serial;
  double data_to_send[8];
  double data_received[8];

  virtual void receiveData(void* data_output, int bytes_expected);
  virtual void sendMessageBegin() = 0;
  virtual void sendMessageEnd() = 0;
  virtual void sendCommandCode(CommandCode code);
  virtual void sendData(void* data, int bytes_to_send);
  virtual void sendCommandMessage(CommandCode code, void* data, int bytes_to_send);

public:
  Transceiver(TxPort* tx, RxPort* rx);
  ~Transceiver();
  ExoMessage* receiveMessages(ExoReport* report);
  void sendReport(ExoReport* report);

};

class MatlabTransceiver:public Transceiver{
public:
  MatlabTransceiver(TxPort* tx, RxPort* rx);
protected:
  void receiveData(void* data_output, int doubles_expected);
  void sendMessageBegin();
  void sendMessageEnd();
  void sendData(void* data, int doubles_to_send);
};

#endif
