#include "Motor.h"
#include "Board.h"
#include "Utils.h"
#include "State_Machine.h"
#include "Auto_KF.h"
#include "Control_Algorithms.h"

Motor::Motor(MotorPins* motor_pins){
  this->motor_pin = motor_pins->motor;
  this->torque_sensor_pin = motor_pins->torque;
  this->motor_error_pin = motor_pins->err;
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

void Motor::setControlAlgorithm(ControlAlgorithm control_algorithm){
  this->previous_control_algorithm = control_algorithm;
  this->control_algorithm = control_algorithm;
}

void Motor::autoKF(int state){
  switch(state){
  case LATE_STANCE:
    Auto_KF_motor_Late_stance(pid_avg_err, pid_setpoint, input);
    break;
  case SWING:
    KF = Auto_KF_motor_Swing(pid_avg_err, KF, kf_clamp);
    break;
  }
}

void Motor::writeToMotor(int value){
  int Vol = this->output + this->zero_torque_reference;

  if (PWM_CONTROL){
    Vol = Vol * 0.8 + 0.1 * 4096.0;
  }
  analogWrite(this->motor_pin, Vol);
}

void Motor::applyPlanterControlAlgorithm(bool taking_baseline, double FSR_percentage, double max_FSR_percentage){
  ControlAlgorithm control_algorithm = this->control_algorithm;

  if(taking_baseline){
    setControlAlgorithm(zero_torque);
  }

  this->setpoint = getControlAlgorithmSetpoint(control_algorithm, this->desired_setpoint,
                                               this->setpoint_clamp, FSR_percentage, max_FSR_percentage);

  if (control_algorithm == 1){
    this->shaping_function->setIterationCount(LATE_STANCE, 1);
  }
}

bool Motor::applyTorque(int state){
  double meas_IMU = 0;
  double torque = this->getTorque();
  double pid_ref;

  //TODO Test IMU balance control
  if (IMU_ENABLED && state == LATE_STANCE && control_algorithm == 2) {
    meas_IMU = imu_clamp.clamp(meas_IMU);
    pid_setpoint = 0;
  } else {
    pid_ref = pid_setpoint;
    input = torque;

    if ((abs(torque) > 25))
    {
      KF = 0;
      return false;

    }
  }

  pid.Compute_KF(KF);
  writeToMotor(output);
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
  in_error_state = digitalRead(motor_error_pin);
}

void Motor::measureTorque(){
  torque_averager->update(this->measureRawCalibratedTorque());
}

double Motor::getTorque(){
  return torque_averager->getAverage();
}

bool Motor::hasErrored(){
  return in_error_state;
}

void Motor::changeState(int state){
  this->old_pid_setpoint = this->pid_setpoint;
  this->new_pid_setpoint = this->previous_setpoint +
    (this->setpoint - this->previous_setpoint) * this->torque_scalar;
}

void Motor::updateSetpoint(int state){
  if ((control_algorithm == 2 || control_algorithm == 3) && state == LATE_STANCE) {
    this->pid_setpoint = this->setpoint;
  } else if (N1 < 1 || N2 < 1 || N3 < 1) {
    this->pid_setpoint = this->new_pid_setpoint;
  } else {
    // Create the smoothed reference and call the PID
    sigmoidCurveSetpoint(state);
  }
}

void Motor::sigmoidCurveSetpoint(int state){
  this->pid_setpoint =
    this->shaping_function->getPIDSetpoint(new_pid_setpoint, pid_setpoint, old_pid_setpoint, state);
}

void Motor::setToZero(){
  this->old_pid_setpoint = this->pid_setpoint;
  this->new_pid_setpoint = 0;
  this->previous_setpoint = 0;
  this->pid_setpoint = 0;
}

void Motor::resetStartingParameters(){
  this->setToZero();
  this->torque_scalar = 0;
  this->shaping_function->setIterationCount(SWING, N1);
  this->shaping_function->setIterationCount(LATE_STANCE, N3);
  this->iter_time_percentage = 0.5;
}

void Motor::updateKFPIDError(){
  error_average->update(pid_setpoint - getTorque());
}

void Motor::applyAutoKF(){
  double err = error_average->getAverage();
  error_average->reset();

  if (err > max_ERR) {
    KF += 0.05;
  }
  else if (err < min_ERR) {
    KF -= 0.05;
  }

  KF = kf_clamp->clamp(KF);
}

void Motor::adjustShapingForTime(double planter_time){
  iter_late_stance = round((planter_time) * iter_time_percentage);
  iter_late_stance = min(500, max(4, N3));
}

void Motor::setTorqueScalar(double scalar){
  scalar = fmin(fmax(scalar,0),1);
  this->torque_scalar = scalar;
}
