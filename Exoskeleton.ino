Exoskeleton::Exoskeleton(){
  left_leg = new Leg();
  right_leg = new Leg();
}

void Exoskeleton::initialize(){
  initialize_left_leg(left_leg);
  initialize_right_leg(right_leg);
}

void Exoskeleton::measureSensors(){
  left_leg->measureSensors();
  right_leg->measureSensors();
}

bool Exoskeleton::checkMotorErrors(){
  return left_leg->checkMotorErrors() || right_leg->checkMotorErrors();
}
