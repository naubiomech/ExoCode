#include "Motor.h"

Motor::Motor(int motor_pin, int torque_sensor_pin, int error_pin){
  this.motor_pin = motor_pin;
  this.torque_sensor_pin = torque_sensor_pin;
  this.motor_error_pin = error_pin;
}

double Motor::measureRawTorque(){
  double Torq = 56.5 / (2.1) * (analogRead(this.torque_sensor_pin) * (3.3 / 4096) - this.torque_calibration_value);
  return -Torq; // TODO Check if negative is necessary
}

void Motor::measureTorque(){

  double average = 0;

  for (int i = dim - 1; i >= 1; i--) {
    torque_measurements[i] = torque_measurements[i - 1];
    average += torque_measurements[j - 1];
  }

  torque_measurements[0] = this.getRawTorque();

  average += torque_measurements[0];
  averaged_torque = average / dim;
}

double Motor::getTorque(){
  return averaged_torque;
}

bool Motor::hasErrored(){
  return digitalRead(exo_motors[i]->err_pin);
}
