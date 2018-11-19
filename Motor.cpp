#include <Arduino.h>
#include "Motor.hpp"
#include "Parameters.hpp"
#include "Board.hpp"
#include "Utils.hpp"
#include "States.hpp"
#include "Control_Algorithms.hpp"
#include "Pins.hpp"
#include "Report.hpp"
#include <PID_v2.h>

Motor::Motor(MotorPins* motor_pins){
  this->motor_pin = motor_pins->motor;
  this->motor_error_pin = motor_pins->err;
  pinMode(motor_error_pin, INPUT);
  pinMode(motor_pin, OUTPUT);
}

void Motor::write(double motor_output){
  int voltage = map((double) motor_output,-1.0,1.0,-2048.0, 2048.0);
  voltage = (output_sign * voltage) + this->zero_torque_reference;
  Serial.println(voltage);

  if (PWM_CONTROL){
    voltage = voltage * 0.8 + 0.1 * 4096.0;
  }
  analogWrite(this->motor_pin, voltage);
}

void Motor::measureError(){
  in_error_state = digitalRead(motor_error_pin);
}

bool Motor::hasErrored(){
  return in_error_state;
}

void Motor::setSign(int sign){
  this->output_sign = (double) sign;
}

MotorReport* Motor::generateReport(){
  MotorReport* report = new MotorReport();
  fillLocalReport(report);
  return report;
}

void Motor::fillReport(MotorReport* report){
  fillLocalReport(report);
}

void Motor::fillLocalReport(MotorReport* report){
  report->error = hasErrored();
}
