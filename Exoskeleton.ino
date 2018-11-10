Exoskeleton::Exoskeleton(ExoPins* exoPins){
  left_leg = new Leg(exoPins->left_leg);
  right_leg = new Leg(exoPins->left_leg);
}

void Exoskeleton::calibrateTorque(){
  left_leg->startTorqueCalibration();
  right_leg->startTorqueCalibration();
  long torque_calibration_value_time = millis();
  while (millis() - torque_calibration_value_time < 1000){
    left_leg->updateTorqueCalibration();
    right_leg->updateTorqueCalibration();
  }
  left_leg->endTorqueCalibration();
  right_leg->endTorqueCalibration();
}

void Exoskeleton::calibrateFSRs(){
  left_leg->calibrateFSRs();
  right_leg->calibrateFSRs();
}

void Exoskeleton::resetStartingParameters(){
  left_leg->resetStartingParameters();
  right_leg->resetStartingParameters();
}

void Exoskeleton::adjustControl(){
  left_leg->adjustControl();
  right_leg->adjustControl();
}

void Exoskeleton::setZeroIfStateState(){
  left_leg->setZeroIfSteadyState();
  right_leg->setZeroIfSteadyState();
}

void Exoskeleton::applyStateMachine(){
  left_leg->applyStateMachine();
  right_leg->applyStateMachine();

}

void Exoskeleton::measureSensors(){
  left_leg->measureSensors();
  right_leg->measureSensors();
}

bool Exoskeleton::checkMotorErrors(){
  return left_leg->checkMotorErrors() || right_leg->checkMotorErrors();
}

void Exoskeleton::disableExo(){
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  stream = 0;
  digitalWrite(LED_PIN, LOW);
}

void Exoskeleton::applyTorque(){
  if(!(left_leg->applyTorque() &&
       right_leg->applyTorque())){
    disableExo();
  }
}
