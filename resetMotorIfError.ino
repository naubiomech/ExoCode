
void resetMotorIfError() {
  static int error_count = 0;
  
  static bool wait_for_reset = false;
  static int local_reset_count = 0;
  
  static bool wait_for_check = false;
  static int wait_count = 0;

  
  if (stream && flag_motor_error_check) {
    Serial.println("IN");
    left_leg->motor_error = !digitalRead(left_leg->pin_err); //digitalRead is pulled HIGH when there is no error
    right_leg->motor_error = !digitalRead(right_leg->pin_err);  //Logical NOT makes it so that an error corresponds to HIGH
    if (left_leg->motor_error) {
      Serial.println("LL Err");
    }
    if (right_leg->motor_error) {
      Serial.println("RL Err");
    }
    bool motor_error = (left_leg->motor_error || right_leg->motor_error);
    if (motor_error && (!wait_for_check && !wait_for_reset)) {
      error_count++;
      Serial.println("Increase error count");
      if (error_count >= ERROR_COUNT_THRESH) {
        Serial.println("Off!");
        digitalWrite(onoff, LOW);
        wait_for_reset = true;
        error_count = 0;
      }
    } else {
      error_count = 0;
    }

    if (wait_for_reset) {
      local_reset_count++;
      Serial.println("Increase reset count");
      if (reset_count >= RESET_COUNT_THRESH) {
        Serial.println("On!");
        digitalWrite(onoff, HIGH);
        wait_for_reset = false;
        wait_for_check = true;
        local_reset_count = 0;
        reset_count++;
      }
    } else {
      local_reset_count = 0;
    }
  
    if (wait_for_check) {
      wait_count++;
      Serial.println("Increase wait count");
      if (wait_count >= WAIT_COUNT_THRESH) {
        wait_for_check = false;
        Serial.println("Wait for check false");
        wait_count = 0;
      }
    } else {
      wait_count = 0;
    }
    
  }//End if(stream...
  
  /*
  //motor_error boolean is "true" if we have an error, false if we don't
  left_leg->motor_error = !digitalRead(left_leg->pin_err); //digitalRead is pulled HIGH when there is no error
  right_leg->motor_error = !digitalRead(right_leg->pin_err);  //Logical NOT makes it so that an error corresponds to HIGH

  (left_leg->motor_error) ? Serial.println("LL Error"):0;
  (right_leg->motor_error) ? Serial.println("RL Error"):0;

  motor_error = (left_leg->motor_error || right_leg->motor_error);

  if (stream == 1 && flag_motor_error_check) {

    //Enable motors for the first time because intially it is set to low
    if (flag_enable_catch_error == true && (digitalRead(onoff) == LOW)) {
      digitalWrite(onoff, HIGH);
    }

    if (motor_error) {
      motor_error_counter++; // motor_error_counter init as 0
      Serial.println("Error!");
    }
    else {
      motor_error_counter = 0;
    }

    if (motor_error_counter >= 2 && flag_enable_catch_error == true) { //flag_enable_catch_error init as true
      turn_off_drivers = true;
      motor_error_counter = 0;
    }

    if (turn_off_drivers) {
      Serial.println("Turn off!");
      digitalWrite(onoff, LOW);
      count_time_drivers_are_off = true;
      flag_enable_catch_error = false;
      turn_off_drivers = false;
    }

    if (count_time_drivers_are_off) {
      time_drivers_are_off++;
    }

    if (time_drivers_are_off >= (16)) {
      Serial.println("Turn on!");
      reset_count++;
      digitalWrite(onoff, HIGH);
      count_time_drivers_are_off = false;
      time_drivers_are_off = 0;
      count_wait_to_allow_for_error_check = true;
    }

    if (count_wait_to_allow_for_error_check) {
      wait_to_allow_for_error_check++;
    }

    if (wait_to_allow_for_error_check >= 20) {
      flag_enable_catch_error = true;
      count_wait_to_allow_for_error_check = false;
      wait_to_allow_for_error_check = 0;
    }


  }//end stream==1
  */
} // resetmotorIfError
