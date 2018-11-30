#include <Arduino.h>
#include "Port.hpp"

int calculateResolution(unsigned int bits){
  int res = 1 << bits;
  return res;
}

Port::Port(unsigned int pin){
  this->pin = pin;
}
Port::~Port(){}

unsigned int Port::getPin(){
  return pin;
}

ImuPort::ImuPort(i2c_pins pins):Port(-1){
  imu_pins = pins;
}

ImuPort::~ImuPort(){

}

i2c_pins ImuPort::getPins(){
  return imu_pins;
}

InputPort::InputPort(unsigned int pin):Port(pin){}
InputPort::~InputPort(){};

AnalogInputPort::AnalogInputPort(unsigned int pin, unsigned int resolution_bits):InputPort(pin){
  resolution = calculateResolution(resolution_bits);
  pinMode(pin, INPUT);
}

AnalogInputPort::~AnalogInputPort(){};

double AnalogInputPort::read(){
  return analogRead(getPin())/resolution;
}

DigitalInputPort::DigitalInputPort(unsigned int pin):InputPort(pin){
  pinMode(pin, INPUT);
}
DigitalInputPort::~DigitalInputPort(){};

double DigitalInputPort::read(){
  return digitalRead(getPin());
}

RxPort::RxPort(unsigned int pin):InputPort(pin){}
RxPort::~RxPort(){};

double RxPort::read(){
  return 0;
}

unsigned int RxPort::getPin(){
  return InputPort::getPin();
}

OutputPort::OutputPort(unsigned int pin):Port(pin){}
OutputPort::~OutputPort(){};

AnalogOutputPort::AnalogOutputPort(unsigned int pin, unsigned int resolution_bits):OutputPort(pin){
  resolution = calculateResolution(resolution_bits);
  pinMode(pin, OUTPUT);
}
AnalogOutputPort::~AnalogOutputPort(){};

void AnalogOutputPort::write(double value){
  analogWrite((int) (value * resolution), getPin());
}

DigitalOutputPort::DigitalOutputPort(unsigned int pin):OutputPort(pin){

  pinMode(pin, OUTPUT);
}
DigitalOutputPort::~DigitalOutputPort(){};

void DigitalOutputPort::write(double value){
  digitalWrite((int) value, getPin());
}

PwmOutputPort::PwmOutputPort(unsigned int pin, unsigned int resolution_bits):AnalogOutputPort(pin, resolution_bits){}
PwmOutputPort::~PwmOutputPort(){};

void PwmOutputPort::write(double value){
  AnalogOutputPort::write(value * 0.8 + 0.1);
}

TxPort::TxPort(unsigned int pin):OutputPort(pin){}
TxPort::~TxPort(){};

void TxPort::write(double value){
}

unsigned int TxPort::getPin(){
  return OutputPort::getPin();
}

PortFactory::~PortFactory(){}

RxPort* ArduinoPortFactory::createRxPort(unsigned int pin){
  return new RxPort(pin);
}

TxPort* ArduinoPortFactory::createTxPort(unsigned int pin){
  return new TxPort(pin);
  }

  InputPort* ArduinoPortFactory::createDigitalInputPort(unsigned int pin){
    return new DigitalInputPort(pin);
  }

  InputPort* ArduinoPortFactory::createAnalogInputPort(unsigned int pin, unsigned int resolution_bits){
    return new AnalogInputPort(pin, resolution_bits);
  }

  OutputPort* ArduinoPortFactory::createDigitalOutputPort(unsigned int pin){
    return new DigitalOutputPort(pin);
  }

  OutputPort* ArduinoPortFactory::createAnalogOutputPort(unsigned int pin, unsigned int resolution_bits){
    return new AnalogOutputPort(pin, resolution_bits);
  }

  OutputPort* ArduinoPortFactory::createPwmOutputPort(unsigned int pin, unsigned int resolution_bits){
    return new PwmOutputPort(pin, resolution_bits);
  }
