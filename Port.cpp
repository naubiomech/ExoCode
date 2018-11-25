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
