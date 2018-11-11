#include <Arduino.h>
#include "Motor.hpp"
#include "Parameters.hpp"
#include "Board.hpp"
#include "Utils.hpp"
#include "State_Machine.hpp"
#include "Control_Algorithms.hpp"
#include "Pins.hpp"
#include "Report.hpp"
#include <PID_v2.h>

Motor::Motor(MotorPins* motor_pins){
  this->motor_pin = motor_pins->motor;
  this->torque_sensor_pin = motor_pins->torque;
  this->motor_error_pin = motor_pins->err;
  this->torque_averager = new MovingAverage(TORQUE_AVERAGE_COUNT);
  this->pid_avg_err = new RunningAverage();
  this->kf_clamp = new Clamp(MAX_KF, MIN_KF);
  this->torque_calibration_average = new RunningAverage();
  this->imu_clamp = new Clamp(-45,45);
  this->shaping_function = new ShapingFunction();
  this->setpoint_clamp = new Clamp(Min_Prop, Max_Prop);
  this->error_average = new RunningAverage();
  pid = new PID(&this->pid_input, &this->pid_output, &this->pid_setpoint,
                PID_DEFAULTS[0], PID_DEFAULTS[1], PID_DEFAULTS[2], REVERSE);
  pid->SetMode(AUTOMATIC);
  pid->SetOutputLimits(-1500, 1500);
  pid->SetSampleTime(PID_sample_time);

  pinMode(this->torque_sensor_pin, INPUT);
  pinMode(motor_error_pin, INPUT);
  pinMode(motor_pin, OUTPUT);
}

void Motor::startTorqueCalibration(){
  torque_calibration_average->reset();
}

void Motor::updateTorqueCalibration(){
  torque_calibration_average->update(measureRawTorque());
}

void Motor::endTorqueCalibration(){
  torque_calibration_value = torque_calibration_average->getAverage();
}

void Motor::setControlAlgorithm(ControlAlgorithm control_algorithm){
  this->previous_control_algorithm = control_algorithm;
  this->control_algorithm = control_algorithm;
}

void Motor::writeToMotor(int value){
  int Vol = this->pid_output + this->zero_torque_reference;

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

  this->setpoint = getSetpoint(control_algorithm, this->desired_setpoint, this->setpoint_clamp,
                               FSR_percentage, max_FSR_percentage, prop_gain);

  if (control_algorithm == 1){
    this->shaping_function->setIterationCount(LATE_STANCE, 1);
  }
}

bool Motor::applyTorque(int state){
  double meas_IMU = 0;
  double torque = this->getTorque();

  //TODO Test IMU balance control
  if (IMU_ENABLED && state == LATE_STANCE && control_algorithm == 2) {
    meas_IMU = imu_clamp->clamp(meas_IMU);
    pid_setpoint = 0;
  } else {
    pid_input = torque;

    if ((abs(torque) > 25)) {
      KF = 0;
      return false;

    }
  }

  pid->Compute_KF(KF);
  writeToMotor(pid_output);
  return true;
}

double Motor::measureRawTorque(){
  double readValue = analogRead(this->torque_sensor_pin);
  return readValue * (3.3 / 4096.0);
}

double Motor::measureRawCalibratedTorque(){
  double rawTorq = measureRawTorque();
  double Torq = 56.5 / (2.1) * (rawTorq - this->torque_calibration_value);
  return Torq; // TODO Check if negative is necessary
}

void Motor::measureError(){
  in_error_state = digitalRead(motor_error_pin);
}

void Motor::measureTorque(){
  double measured = this->measureRawCalibratedTorque();
  torque_averager->update(measured);
}

double Motor::getTorque(){
  double torque = torque_averager->getAverage();
  return torque;
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
  } else if (iter_late_stance < 1 || iter_swing < 1) {
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
  this->shaping_function->setIterationCount(SWING, DEFAULT_ITER_SWING);
  this->shaping_function->setIterationCount(LATE_STANCE, DEFAULT_ITER_LATE_STANCE);
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
  if(adjust_shaping_for_time){
    iter_late_stance = round((planter_time) * iter_time_percentage);
    iter_late_stance = min(500, max(4, iter_late_stance));
  }
}

void Motor::setTorqueScalar(double scalar){
  scalar = fmin(fmax(scalar,0),1);
  this->torque_scalar = scalar;
}

MotorReport Motor::generateReport(){
  MotorReport report = MotorReport();
  report.measuredTorque = getTorque();
  report.pid_setpoint = pid_setpoint;
  return report;
}

void Motor::fillReport(MotorReport* report){
  report->measuredTorque = getTorque();
  report->pid_setpoint = pid_setpoint;
}
