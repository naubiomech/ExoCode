// functions used for the Leg objects

void initialize_leg(Leg* leg) {
  pinMode(leg->pin_err, INPUT_PULLUP);          //motor driver error checking needs digital input pulled high
  pinMode(leg->torque_sensor_ankle_pin, INPUT); //enable the torque reading of the left torque sensor
  pinMode(leg->motor_current_pin, INPUT);
  pinMode(leg->motor_speed_pin,INPUT);
  pinMode(leg->ankle_angle_pin,INPUT);
  

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
  left_leg->fsr_sense_Heel = FSR_SENSE_LEFT_HEEL_PIN;
  left_leg->fsr_sense_Toe = FSR_SENSE_LEFT_TOE_PIN;
  left_leg->torque_address = 0;
  left_leg->address_FSR = 18;
  left_leg->p_steps = &val_L;
  left_leg->torque_sensor_ankle_pin = TORQUE_SENSOR_LEFT_ANKLE_PIN;
  left_leg->motor_ankle_pin = MOTOR_LEFT_ANKLE_PIN;
  left_leg->motor_current_pin = MOTOR_CURRENT_LEFT_ANKLE_PIN;
  left_leg->baseline_address = address_params + 105 + 5;
  left_leg->angle_address = 192;
  if (check_angle_bias(left_leg->angle_address)) { //If a saved angle calibration exists
    left_leg->angle_zero = read_angle_bias(left_leg->angle_address);
    //Serial.println("Successful angle bias update");
    //Serial.println(left_leg->angle_zero);
  } else {
    left_leg->angle_zero = 0;
  }
  left_leg->motor_speed_pin = MOTOR_SPEED_LEFT_PIN;
  left_leg->ankle_angle_pin = HALL_LEFT_PIN;
  left_leg->potentiometer_pin = HALL_LEFT_PIN;

  left_leg->Dynamic_multiplier = 1;
  left_leg->Steady_multiplier = 1;


  initialize_leg(left_leg);


  left_leg->whos = 'L';
}

void initialize_right_leg(Leg* right_leg) {
  right_leg->pin_err = MOTOR_ERROR_RIGHT_ANKLE_PIN;
  right_leg->fsr_sense_Heel = FSR_SENSE_RIGHT_HEEL_PIN;
  right_leg->fsr_sense_Toe = FSR_SENSE_RIGHT_TOE_PIN;
  right_leg->torque_address = 9;
  right_leg->address_FSR = 36;
  right_leg->p_steps = &val_R;
  right_leg->torque_sensor_ankle_pin = TORQUE_SENSOR_RIGHT_ANKLE_PIN;
  right_leg->motor_ankle_pin = MOTOR_RIGHT_ANKLE_PIN;
  right_leg->motor_current_pin = MOTOR_CURRENT_RIGHT_ANKLE_PIN;
  right_leg->baseline_address = address_params + 105 + 5 + 9;
  right_leg->angle_address = 201;
  if (check_angle_bias(right_leg->angle_address)) { //If a saved angle calibration exists
    right_leg->angle_zero = read_angle_bias(right_leg->angle_address);
  } else {
    right_leg->angle_zero = 0;
  }
    
  right_leg->motor_speed_pin = MOTOR_SPEED_RIGHT_PIN;
  right_leg->ankle_angle_pin = HALL_RIGHT_PIN;
  right_leg->potentiometer_pin = HALL_RIGHT_PIN;



  right_leg->Dynamic_multiplier = 1;
  right_leg->Steady_multiplier = 1;

  initialize_leg(right_leg);

  right_leg->whos = 'R';
}
