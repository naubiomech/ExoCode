Exoskeleton::Exoskeleton(){
  left_leg = new Leg();
  right_leg = new Leg();
}

void Exoskeleton::setZeroIfStateState(){
  left_leg->setZeroIfSteadyState();
  right_leg->setZeroIfSteadyState();
}

void Exoskeleton::applyStateMachine(){
  left_leg->applyStateMachine();
  right_leg->applyStateMachine();

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

void Exoskeleton::takeFSRBaseline(){
  right_leg->takeBaseline();
  left_leg->takeBaseline();
}

void Exoskeleton::disableExo(){
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
    stream = 0;
    digitalWrite(LED_PIN, LOW);
    leg->state = old_L_state_L;
  }

  void Exoskeleton::applyTorque(){
    if(!(left_leg->applyTorque() &&
         right_leg->applyTorque())){
      disableExo();
    }
  }
