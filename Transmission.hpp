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
  float* send_data;
  double* receive_data;

  virtual void processData(ExoMessageBuilder* builder, ExoReport* report) = 0;
  virtual void preprocessData();
  virtual void postprocessData();
  void copyToSend(double* from);
  void copyFromReceive(double* to);
public:
  Transmission(Transceiver* transceiver, CommandCode code,
               unsigned int receive_count, unsigned int send_count);
	virtual ~Transmission();
	void process(ExoMessageBuilder* builder, ExoReport* report);
};

class JointSelectTransmission:public Transmission{
private:
  TeensyByteTranscriber byte_transcriber;
  static const int select_count = 3;
  unsigned int send_count;
  unsigned int receive_count;
	bool send_select;
	bool receive_select;

	void separateSelect(int* select, double* receive);
  void combineSelect(int* select, double* send);
  void decodeJointSelect(int* selects, double encoded_select);
  float encodeJointSelect(int* selects);

protected:
  unsigned int selects[3];
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report) = 0;
  virtual void preprocessData();
  virtual void postprocessData();
public:
	JointSelectTransmission(Transceiver* transceiver, CommandCode code,
							unsigned int receive_count, bool receive_select,
							unsigned int send_count, bool send_select);
};

class RequestDataTransmission:public Transmission{
public:
  explicit RequestDataTransmission(Transceiver* transceiver);
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
  explicit EndTrialTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class CalibrateTorqueTransmission:public Transmission{
public:
  explicit CalibrateTorqueTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class CheckBluetoothTransmission:public Transmission{
public:
  explicit CheckBluetoothTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class CleanBluetoothBufferTransmission:public Transmission{
public:
  explicit CleanBluetoothBufferTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetSetpointTransmission:public JointSelectTransmission{
public:
  explicit GetSetpointTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class SetSetpointTransmission:public JointSelectTransmission{
public:
  explicit SetSetpointTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class CalibrateFsrTransmission:public Transmission{
public:
  explicit CalibrateFsrTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetFsrThresholdTransmission:public Transmission{
public:
  explicit GetFsrThresholdTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetKFTransmission:public JointSelectTransmission{
public:
  explicit GetKFTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class SetKFTransmission:public JointSelectTransmission{
public:
  explicit SetKFTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetPidParamsTransmission:public JointSelectTransmission{
public:
  explicit GetPidParamsTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class SetPidParamsTransmission:public JointSelectTransmission{
public:
  explicit SetPidParamsTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class GetSmoothingParamsTransmission:public JointSelectTransmission{
public:
  explicit GetSmoothingParamsTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class SetSmoothingParamsTransmission:public JointSelectTransmission{
public:
  explicit SetSmoothingParamsTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};

class CheckMemoryTransmission:public Transmission{
public:
  explicit CheckMemoryTransmission(Transceiver* transceiver);
private:
  virtual void processData(ExoMessageBuilder* builder, ExoReport* report);
};


class TransmissionFactory{
public:
  Transmission* create(Transceiver* transceiver, CommandCode code);
};
#endif
