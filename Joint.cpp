#include "Joint.hpp"
#include "Board.hpp"

Joint::Joint(JointPins* pins){
  motor = new Motor(pins->motor_pins);
  torque_sensor = new TorqueSensor(pins->torque_sensor_pins);
  this->kf_clamp = new Clamp(MAX_KF, MIN_KF);
  this->imu_clamp = new Clamp(-45,45);
  this->shaping_function = new ShapingFunction();
  this->setpoint_clamp = new Clamp(Min_Prop, Max_Prop);
  this->error_average = new RunningAverage();

  pid = new PID(&this->pid_input, &this->pid_output, &this->pid_setpoint,
                PID_DEFAULTS[0], PID_DEFAULTS[1], PID_DEFAULTS[2], REVERSE);
  pid->SetMode(AUTOMATIC);
  pid->SetOutputLimits(-1, 1);
  pid->SetSampleTime(PID_sample_time);

}

void Joint::measureError(){
  motor->measureError();
}

bool Joint::hasErrored(){
  return motor->hasErrored();
}

bool Joint::applyTorque(int state){
  double torque = torque_sensor->getTorque();

  //TODO Test IMU balance control
  if (IMU_ENABLED && state == LATE_STANCE && control_algorithm->getType() == balance_control) {
    pid_setpoint = 0;
  } else {
    pid_input = torque;

    if ((abs(torque) > 25)) {
      KF = 0;
      return false;

    }
  }
  pid->Compute_KF(KF);
  motor->write(pid_output);
  return true;
}

void Joint::scaleSetpointDifference(){
  this->old_pid_setpoint = this->pid_setpoint;
  this->new_pid_setpoint = this->previous_setpoint +
    (this->setpoint - this->previous_setpoint) * this->torque_scalar;
}

void Joint::updateSetpoint(int state){
  if ((control_algorithm->getType() == proportional || control_algorithm->getType() == pivot_proportional) && state == LATE_STANCE) {
    this->pid_setpoint = this->setpoint;
  } else {
    // Create the smoothed reference and call the PID
    sigmoidCurveSetpoint(state);
  }
}

void Joint::sigmoidCurveSetpoint(int state){
  this->pid_setpoint =
    this->shaping_function->getPIDSetpoint(new_pid_setpoint, pid_setpoint, old_pid_setpoint, state);
}

void Joint::setToZero(){
  this->old_pid_setpoint = this->pid_setpoint;
  this->new_pid_setpoint = 0;
  this->previous_setpoint = 0;
  this->pid_setpoint = 0;
}

void Joint::resetStartingParameters(){
  this->setToZero();
  this->torque_scalar = 0;
  this->shaping_function->setIterationCount(SWING, DEFAULT_ITER_SWING);
  this->shaping_function->setIterationCount(LATE_STANCE, DEFAULT_ITER_LATE_STANCE);
  this->iter_time_percentage = 0.5;
}

void Joint::adjustShapingForTime(double planter_time){
  if(adjust_shaping_for_time){
    double iter_late_stance;
    iter_late_stance = round((planter_time) * iter_time_percentage);
    iter_late_stance = min(500, max(4, iter_late_stance));

    this->shaping_function->setIterationCount(LATE_STANCE, iter_late_stance);
  }
}

void Joint::setTorqueScalar(double scalar){
  scalar = fmin(fmax(scalar,0),1);
  this->torque_scalar = scalar;
}

void Joint::adjustSetpoint(double FSR_percentage, double max_FSR_percentage){
  this->setpoint = this->control_algorithm->getSetpoint(FSR_percentage, max_FSR_percentage);

  if (control_algorithm->getType() == balance_control){
    this->shaping_function->setIterationCount(LATE_STANCE, 1);
  }
}

void Joint::updateKFPIDError(){
  double torque = torque_sensor->getTorque();
  error_average->update(pid_setpoint - torque);
}

void Joint::applyAutoKF(){
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

double Joint::getTorque(){
  return torque_sensor->getTorque();
}

void Joint::measureTorque(){
  torque_sensor->measureTorque();
}

void Joint::startTorqueCalibration(){
  torque_sensor->startTorqueCalibration();
}

void Joint::updateTorqueCalibration(){
  torque_sensor->updateTorqueCalibration();
}

void Joint::endTorqueCalibration(){
  torque_sensor->endTorqueCalibration();
}

void Joint::setSign(int sign){
  motor->setSign(sign);
  torque_sensor->setSign(sign);
}

JointReport* Joint::generateReport(){
  JointReport* report = new JointReport();
  fillLocalReport(report);
  report->motor_report = motor->generateReport();
  report->torque_sensor_report = torque_sensor->generateReport();
  return report;
}

void Joint::fillReport(JointReport* report){
  fillLocalReport(report);
  motor->fillReport(report->motor_report);
  torque_sensor->fillReport(report->torque_sensor_report);
}

void Joint::fillLocalReport(JointReport* report){
  report->pid_setpoint = pid_setpoint;

}
