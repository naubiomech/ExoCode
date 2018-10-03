Exoskeleton::Exoskeleton(){
  left_leg = new Leg();
  right_leg = new Leg();
}

void Exoskeleton::measureSensors(){
  left_leg->measureSensors();
  right_leg->measureSensors();
}

bool Exoskeleton::checkMotorErrors(){
  return left_leg->checkMotorErrors() || right_leg->checkMotorErrors();
}
