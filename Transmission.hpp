#ifndef TRANSMISSION_HEADER
#define TRANSMISSION_HEADER
#include "Commands.hpp"
#include "Command_Codes.hpp"
#include "Message.hpp"
#include "Transceiver.hpp"
#include "Report.hpp"
#include "JointSelect.hpp"


class Transmission{
private:
  unsigned int send_count;
  unsigned int receive_count;
  CommandCode code;

  void getData();
  void sendData();
protected:
  Transceiver* transceiver;
  double* send_data;
  double* receive_data;

  virtual void processData(ExoMessageBuilder* builder, ExoReport* report) = 0;
  void copyToSend(double* from);
  void copyFromReceive(double* to);
  void decodeJointSelect(int* selects, double encoded_select);
public:
  Transmission(Transceiver* transceiver, CommandCode code,
               unsigned int receive_count, unsigned int send_count);
  virtual ~Transmission();
  void process(ExoMessageBuilder* builder, ExoReport* report);
  TeensyByteTranscriber byte_transcriber;
};

class RequestDataTransmission:public Transmission{
public:
  RequestDataTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class StartTrialTransmission:public Transmission{
public:
  StartTrialTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class EndTrialTransmission:public Transmission{
public:
  EndTrialTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class CalibrateTorqueTransmission:public Transmission{
public:
  CalibrateTorqueTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class CheckBluetoothTransmission:public Transmission{
public:
  CheckBluetoothTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class CleanBluetoothBufferTransmission:public Transmission{
public:
  CleanBluetoothBufferTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetSetpointTransmission:public Transmission{
public:
  GetSetpointTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class SetSetpointTransmission:public Transmission{
public:
  SetSetpointTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class CalibrateFsrTransmission:public Transmission{
public:
  CalibrateFsrTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetFsrThresholdTransmission:public Transmission{
public:
  GetFsrThresholdTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetKFTransmission:public Transmission{
public:
  GetKFTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class SetKFTransmission:public Transmission{
public:
  SetKFTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetPidParamsTransmission:public Transmission{
public:
  GetPidParamsTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class SetPidParamsTransmission:public Transmission{
public:
  SetPidParamsTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetSmoothingParamsTransmission:public Transmission{
public:
  GetSmoothingParamsTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class SetSmoothingParamsTransmission:public Transmission{
public:
  SetSmoothingParamsTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class CheckMemoryTransmission:public Transmission{
public:
  CheckMemoryTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};


class TransmissionFactory{
public:
  Transmission* create(Transceiver* transceiver, CommandCode code);
};
#endif
