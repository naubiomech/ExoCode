#include "Motor.h"
#include "Board.h"
#include "Utils.h"
#include "State_Machine.h"
#include "Auto_KF.h"

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

void Motor::applyPlanterControlAlgorithm(double FSR_percentage, int set_zero_flag, double max_FSR_percentage, int control_algorithm){
  int iter_late_stance = this->shaping_function->getIterationCount(LATE_STANCE);
  int new_iters;
  if (FSR_baseline_FLAG){
    Setpoint = 0;
    Setpoint_Pctrl = 0;
    new_iters = iter_late_stance;
  } else if (control_algorithm != 0){
    *p_Setpoint = Ctrl_ADJ_planter_general(control_algorithm, desired_setpoint, setpoint_clamp,
                                           FSR_percentage, max_FSR_percentage, control_algorithm);
    if (control_algorithm == 1){
      iter_late_stance = 1;
    }
  }
  this->shaping_function->setIterationCount(LATE_STANCE, new_iters);
}


void Motor::changeState(int state){
  this->Old_PID_Setpoint = this->PID_Setpoint;
  this->New_PID_Setpoint = this->Previous_Setpoint +
    (this->Setpoint - this->Previous_Setpoint) * this->coef_in_3_steps;
  if (state == LATE_STANCE){
    if (abs(this->Dorsi_Setpoint) > 0) {
      this->Old_PID_Setpoint = 0;
    } else {
      this->Previous_Dorsi_Setpoint = 0;
    }
  }
}

void Motor::updateSetpoint(int state){
  if ((Trq_time_volt == 2 || Trq_time_volt == 3) && state == LATE_STANCE) {
    this->PID_Setpoint = this->Setpoint_Pctrl;
  } else if (N1 < 1 || N2 < 1 || N3 < 1) {
    this->PID_Setpoint = this->New_PID_Setpoint;
  } else {
    // Create the smoothed reference and call the PID
    sigmoidCurveSetpoint(state);
  }
}

void Motor::sigmoidCurveSetpoint(int state){
  long sig_time = millis();
  if ((sig_time - this->sig_time_old) > 1) {

    this->sig_time_old = sig_time;
    this->PID_Setpoint =
      this->shaping_function->getPIDSetpoint(New_PID_Setpoint, PID_Setpoint, Old_PID_Setpoint, state);
  }
}

void Motor::setToZero(){
  this->Old_PID_Setpoint = this->PID_Setpoint;
  this->New_PID_Setpoint = 0;
  this->Previous_Setpoint = 0;
  this->PID_Setpoint = 0;
  this->Setpoint_Pctrl = 0;
}

void Motor::resetStartingParameters(){

  this->Previous_Setpoint = 0;
  this->coef_in_3_steps = 0;

  this->perc_l = 0.5;
}

void Motor::adjustShapingForTime(double planterTime){
  iter_late_stance = round((this->plant_mean) * iter_time_percentage);
  iter_late_stance = min(500, max(4, N3));
}
