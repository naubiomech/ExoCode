#ifndef PORT_HEADER
#define PORT_HEADER

#include <i2c_t3.h>

class Port{
private:
  unsigned int pin;
protected:
  unsigned int getPin();
public:
  Port(unsigned int pin);
};

class ImuPort:public Port{
private:
  i2c_pins imu_pins;
  ImuPort(i2c_pins imu_pins);
public:
  i2c_pins getPins();
};

class InputPort:public Port{
public:
  InputPort(unsigned int pin);
  virtual double read() = 0;
};

class AnalogInputPort: public InputPort{
private:
  int resolution;
public:
  double read();
  AnalogInputPort(unsigned int pin, unsigned int resolution_bits);
};

class DigitalInputPort: public InputPort{
public:
  double read();
  DigitalInputPort(unsigned int pin);
};

class RxPort:public InputPort{
public:
  double read();
  unsigned int getPin();
  RxPort(unsigned int pin);
};

class OutputPort: public Port{
private:
  int resolution;
public:
  virtual void write(double value) = 0;
  OutputPort(unsigned int pin);
};

class AnalogOutputPort: public OutputPort{
private:
  int resolution;
public:
  void write(double value);
  AnalogOutputPort(unsigned int pin, unsigned int bit_resolution);
};

class DigitalOutputPort: public OutputPort{
public:
  void write(double value);
  DigitalOutputPort(unsigned int pin);
};

class PwmOutputPort: public AnalogOutputPort{
public:
  void write(double value);
  PwmOutputPort(unsigned int pin, unsigned int resolution_bits);
};

class TxPort:public OutputPort{
public:
  void write(double value);
  unsigned int getPin();
  TxPort(unsigned int pin);
};

class PortFactory{
public:
  virtual RxPort* createRxPort(unsigned int pin) = 0;
  virtual TxPort* createTxPort(unsigned int pin) = 0;
  virtual InputPort* createDigitalInputPort(unsigned int pin) = 0;
  virtual InputPort* createAnalogInputPort(unsigned int pin, unsigned int resolution_bits) = 0;
  virtual OutputPort* createDigitalOutputPort(unsigned int pin) = 0;
  virtual OutputPort* createAnalogOutputPort(unsigned int pin, unsigned int resolution_bits) = 0;
  virtual OutputPort* createPwmOutputPort(unsigned int pin, unsigned int resolution_bits) = 0;
};

class ArduinoPortFactory:public PortFactory{
  RxPort* createRxPort(unsigned int pin);
  TxPort* createTxPort(unsigned int pin);
  InputPort* createDigitalInputPort(unsigned int pin);
  InputPort* createAnalogInputPort(unsigned int pin, unsigned int resolution_bits);
  OutputPort* createDigitalOutputPort(unsigned int pin);
  OutputPort* createAnalogOutputPort(unsigned int pin, unsigned int resolution_bits);
  OutputPort* createPwmOutputPort(unsigned int pin, unsigned int resolution_bits);
};

#endif
