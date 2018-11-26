#include <Arduino.h>
#include "Port.hpp"

int calculateResolution(unsigned int bits){
  int res = 1 << bits;
  return res;
}

Port::Port(unsigned int pin){
  this->pin = pin;
}

unsigned int Port::getPin(){
  return pin;
}

InputPort::InputPort(unsigned int pin):Port(pin){}

AnalogInputPort::AnalogInputPort(unsigned int pin, unsigned int resolution_bits):InputPort(pin){
  resolution = calculateResolution(resolution_bits);
  pinMode(pin, INPUT);
}

double AnalogInputPort::read(){
  return analogRead(getPin())/resolution;
}

DigitalInputPort::DigitalInputPort(unsigned int pin):InputPort(pin){
  pinMode(pin, INPUT);
}

double DigitalInputPort::read(){
  return digitalRead(getPin());
}

OutputPort::OutputPort(unsigned int pin):Port(pin){}

AnalogOutputPort::AnalogOutputPort(unsigned int pin, unsigned int resolution_bits):OutputPort(pin){
  resolution = calculateResolution(resolution_bits);
  pinMode(pin, OUTPUT);
}

void AnalogOutputPort::write(double value){
  analogWrite((int) (value * resolution), getPin());
}

DigitalOutputPort::DigitalOutputPort(unsigned int pin):OutputPort(pin){

  pinMode(pin, OUTPUT);
}

void DigitalOutputPort::write(double value){
  digitalWrite((int) value, getPin());
}

PwmOutputPort::PwmOutputPort(unsigned int pin, unsigned int resolution_bits):AnalogOutputPort(pin, resolution_bits){}

void PwmOutputPort::write(double value){
  AnalogOutputPort::write(value * 0.8 + 0.1);
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
