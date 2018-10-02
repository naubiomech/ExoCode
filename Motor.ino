#include "Motor.h"

Motor::Motor(int motor_pin, int torque_sensor_pin, int error_pin){
  this.motor_pin = motor_pin;
  this.torque_sensor_pin = torque_sensor_pin;
  this.motor_error_pin = error_pin;
}
