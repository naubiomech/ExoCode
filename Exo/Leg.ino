// functions used for the Leg objects

void initialize_leg(Leg* leg) {
  
  pinMode(leg->pin_err, INPUT_PULLUP);          //motor driver error checking needs digital input pulled high
  pinMode(leg->torque_sensor_ankle_pin, INPUT); //enable the torque reading of the left torque sensor
  pinMode(leg->motor_current_pin, INPUT);

  analogWrite(leg->motor_ankle_pin, zero);
  leg->pid.SetMode(AUTOMATIC);
  leg->pid.SetTunings(leg->kp, leg->ki, leg->kd);                                      //Kp, Ki, Kd ##COULD BE AUTOTUNED
  leg->pid.SetOutputLimits(-4500, 4500);                                  //range of Output around 0 ~ 1995 ##THIS IS DIFFERENT NOW AND SHOULD CONCRETELY CONFIRM
  
  leg->pid.SetSampleTime(PID_sample_time);

  leg->p_steps->fsr_Toe = leg->fsr_sense_Toe;
  leg->zero = zero;
}

void initialize_left_leg(Leg* left_leg) {

  left_leg->fsr_sense_Toe = FSR_SENSE_LEFT_TOE_PIN;
  left_leg->p_steps = &val_L;
  left_leg->torque_sensor_ankle_pin = TORQUE_SENSOR_LEFT_ANKLE_PIN;
  
  left_leg->Dynamic_multiplier = 1;
  left_leg->Steady_multiplier = 1;


  initialize_leg(left_leg);

  left_leg->whos = 'L';
}

void initialize_right_leg(Leg* right_leg) {
  right_leg->fsr_sense_Toe = FSR_SENSE_RIGHT_TOE_PIN;
  right_leg->p_steps = &val_R;
  right_leg->torque_sensor_ankle_pin = TORQUE_SENSOR_RIGHT_ANKLE_PIN;

  right_leg->Dynamic_multiplier = 1;
  right_leg->Steady_multiplier = 1;

  initialize_leg(right_leg);

  right_leg->whos = 'R';
}
