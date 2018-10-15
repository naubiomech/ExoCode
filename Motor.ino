#include "Motor.h"
#include "Utils.h"
#include "State_Machine.h"
#include "Auto_KF.h"

Motor::Motor(int motor_pin, int torque_sensor_pin, int error_pin){
  this->motor_pin = motor_pin;
  this->torque_sensor_pin = torque_sensor_pin;
  this->motor_error_pin = error_pin;
}

void Motor::startTorqueCalibration(){
  torque_calibration_average->reset();
}

void Motor::updateTorqueCalibration(){
  torque_calibration_average->update(measureRawTorque());
}

void Motor::endTorqueCalibration(){
  torque_calibration_value = torque_calibration_average->getAverage() * (3.3/4096.0);
}

void Motor::autoKF(int state){
  switch(state){
  case LATE_STANCE:
    Auto_KF_motor_Late_stance(pid_avg_err, PID_Setpoint, Input);
    break;
  case SWING:
    KF = Auto_KF_motor_Swing(pid_avg_err, KF, kf_clamp);
    break;
  }
}

void Motor::writeToMotor(int value){
  int Vol = this->Output + this->zero_torque_reference;

  if (PWM_CONTROL){
    Vol = Vol * 0.8 + 0.1 * 4096.0;
  }
  analogWrite(this->motor_pin, Vol);
}

bool Motor::applyTorque(int state){
  double torque = averaged_torque;
  double PID_ref;

  //TODO Test IMU balance control
  if (IMU_ENABLED && state == LATE_STANCE && Trq_time_volt == 2) {
    meas_IMU = imu_clamp.clamp(meas_IMU);
    PID_Setpoint = 0;
    Input = meas_IMU * IMU_Gain;
  } else {
    PID_ref = PID_Setpoint;
    Input = torque;

    if ((abs(torque) > 25))
    {
      KF = 0;
      return false;

    }
  }

  pid.Compute_KF(KF);
  writeToMotor(Output);
  return true;
}

double Motor::measureRawTorque(){
  return analogRead(this->torque_sensor_pin) * (3.3 / 4096);
}

double Motor::measureRawCalibratedTorque(){
  double Torq = 56.5 / (2.1) * (measureRawCalibratedTorque() - this->torque_calibration_value);
  return -Torq; // TODO Check if negative is necessary
}

void Motor::measureError(){
  inErrorState = digitalRead(motor_error_pin);
}

void Motor::measureTorque(){

  double average = 0;

  for (int i = TORQUE_AVERAGE_COUNT - 1; i >= 1; i--) {
    torque_measurements[i] = torque_measurements[i - 1];
    average += torque_measurements[i - 1];
  }

  torque_measurements[0] = this->measureRawCalibratedTorque();

  average += torque_measurements[0];
  averaged_torque = average / TORQUE_AVERAGE_COUNT;
}

double Motor::getTorque(){
  return averaged_torque;
}

bool Motor::hasErrored(){
  return inErrorState;
}
