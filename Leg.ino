// functions used for the Leg objects

void initialize_leg(Leg* leg) {
  
  pinMode(leg->pin_err, INPUT_PULLUP);          //motor driver error checking needs digital input pulled high

  analogWrite(leg->motor_ankle_pin, zero);
  leg->pid.SetMode(AUTOMATIC);
  leg->pid.SetTunings(leg->kp, leg->ki, leg->kd);                                      //Kp, Ki, Kd ##COULD BE AUTOTUNED
  leg->pid.SetOutputLimits(-1500, 1500);                                  //range of Output around 0 ~ 1995 ##THIS IS DIFFERENT NOW AND SHOULD CONCRETELY CONFIRM
  leg->pid.SetSampleTime(PID_sample_time);

  leg->p_steps->fsr_Toe = leg->fsr_sense_Toe;
  leg->zero = zero;
}

void initialize_left_leg(Leg* left_leg) {
  left_leg->pin_err = MOTOR_ERROR_LEFT_ANKLE_PIN;
  //left_leg->fsr_sense_Heel = FSR_SENSE_LEFT_HEEL_PIN;
  left_leg->fsr_sense_Toe = FSR_SENSE_LEFT_TOE_PIN;
  //left_leg->torque_address = 0;
  //left_leg->address_FSR = 18;
  left_leg->p_steps = &val_L;
  left_leg->motor_ankle_pin = MOTOR_LEFT_ANKLE_PIN;

  left_leg->Dynamic_multiplier = 1;
  left_leg->Steady_multiplier = 1;


  initialize_leg(left_leg);
  pinMode(TORQUE_SENSOR_LEFT_ANKLE_PIN, INPUT); //enable the torque reading of the left torque sensor
  pinMode(MOTOR_CURRENT_LEFT_ANKLE_PIN, INPUT);


  left_leg->whos = 'L';
}

void initialize_right_leg(Leg* right_leg) {
  right_leg->pin_err = MOTOR_ERROR_RIGHT_ANKLE_PIN;
  //right_leg->fsr_sense_Heel = FSR_SENSE_RIGHT_HEEL_PIN;
  right_leg->fsr_sense_Toe = FSR_SENSE_RIGHT_TOE_PIN;
  //right_leg->torque_address = 9;
  //right_leg->address_FSR = 36;
  right_leg->p_steps = &val_R;
  right_leg->motor_ankle_pin = MOTOR_RIGHT_ANKLE_PIN;


  right_leg->Dynamic_multiplier = 1;
  right_leg->Steady_multiplier = 1;

  initialize_leg(right_leg);
  pinMode(TORQUE_SENSOR_RIGHT_ANKLE_PIN, INPUT); //enable the torque reading of the left torque sensor
  pinMode(MOTOR_CURRENT_RIGHT_ANKLE_PIN, INPUT);

  right_leg->whos = 'R';
  //right_leg->Potentiometer_pin = A17;
}
