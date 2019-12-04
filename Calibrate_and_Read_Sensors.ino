// Calibrate the torque sensors, FSR sensors and get the torque and FSR voltage

void torque_calibration()
{
  long torque_calibration_value_time = millis();
  int torq_cal_count = 0;
  left_leg->torque_calibration_value = 0;
  right_leg->torque_calibration_value = 0;

  while (millis() - torque_calibration_value_time < 1000)
  { //Calibrates the LL for a total time of 1 second,
    left_leg->torque_calibration_value += analogRead(TORQUE_SENSOR_LEFT_ANKLE_PIN) * (3.3 / 4096);                                        //Sums the torque read in and sums it with all previous red values
    right_leg->torque_calibration_value += analogRead(TORQUE_SENSOR_RIGHT_ANKLE_PIN) * (3.3 / 4096);
    torq_cal_count ++;                                                         //Increments count
  }

  left_leg->torque_calibration_value = left_leg->torque_calibration_value / torq_cal_count;                       // Averages torque over a second
  right_leg->torque_calibration_value = right_leg->torque_calibration_value / torq_cal_count;                       // Averages torque over a second
  //  Serial.println(left_leg->torque_calibration_value);
  //  Serial.println(right_leg->torque_calibration_value);
}



void FSR_calibration()
{


  if (FSR_FIRST_Cycle) {
    FSR_FIRST_Cycle = 0;

    startTime = millis();
    //    Serial.println("First time");
    right_leg->Curr_Combined = 0;
    left_leg->Curr_Combined = 0;

    right_leg->fsr_Combined_peak_ref = 0;
    left_leg->fsr_Combined_peak_ref = 0;
    left_leg->fsr_Toe_peak_ref = 0;
    right_leg->fsr_Toe_peak_ref = 0;
    left_leg->fsr_Heel_peak_ref = 0;
    right_leg->fsr_Heel_peak_ref = 0;
  }


  if (millis() - startTime < 5000)
  {
    left_leg->Curr_Toe = fsr(left_leg->fsr_sense_Toe);
    right_leg->Curr_Toe = fsr(right_leg->fsr_sense_Toe);

    left_leg->Curr_Heel = fsr(left_leg->fsr_sense_Heel);
    right_leg->Curr_Heel = fsr(right_leg->fsr_sense_Heel);

    left_leg->Curr_Combined = left_leg->Curr_Toe + left_leg->Curr_Heel;
    right_leg->Curr_Combined = right_leg->Curr_Toe + right_leg->Curr_Heel;

    if (left_leg->Curr_Combined > left_leg->fsr_Combined_peak_ref)
    {
      left_leg->fsr_Combined_peak_ref = left_leg->Curr_Combined;
    }

    if (right_leg->Curr_Combined > right_leg->fsr_Combined_peak_ref)
    {
      right_leg->fsr_Combined_peak_ref = right_leg->Curr_Combined;
    }

    // Toe

    if (left_leg->Curr_Toe > left_leg->fsr_Toe_peak_ref)
    {
      left_leg->fsr_Toe_peak_ref = left_leg->Curr_Toe;
    }

    if (right_leg->Curr_Toe > right_leg->fsr_Toe_peak_ref)
    {
      right_leg->fsr_Toe_peak_ref = right_leg->Curr_Toe;
    }

    // Heel
    if (left_leg->Curr_Heel > left_leg->fsr_Heel_peak_ref)
    {
      left_leg->fsr_Heel_peak_ref = left_leg->Curr_Heel;
    }

    if (right_leg->Curr_Heel > right_leg->fsr_Heel_peak_ref)
    {
      right_leg->fsr_Heel_peak_ref = right_leg->Curr_Heel;
    }

  }
  else {

    FSR_FIRST_Cycle = 1;
    FSR_CAL_FLAG = 0;
  }
}

double get_torq(Leg* leg) {
  double Torq = 56.5 / (2.1) * (analogRead(leg->torque_sensor_ankle_pin) * (3.3 / 4096) - leg->torque_calibration_value);
  return -Torq;             //neg is here for right leg, returns the torque value of the right leg (Newton-Meters)
}

double get_LL_torq()
{ //flexion is positive 8.10.16, gets the torque of the right leg
  return get_torq(left_leg);
}

double get_RL_torq()
{ //flexion is positive 8.10.16, gets the torque of the right leg
  return get_torq(right_leg);
}

/*FSR Code
	This code is very basic but is kept as an outside function for clarity. The FSR readings are used to control state based actions based on the part of the gait cycle the patient
	is in.
*/
double fsr(const unsigned int pin) {
  //using voltage divider: 3.3 V -- >FSR<-- Vo -- >R< (1000) -- ground
  //Vo from analog read: 3.3* analog read/ max read (4096) || I = Vo/R || FSR resistance = (3.3V - Vo)/I
  double Vo = 10 * 3.3 * analogRead(pin) / 4096; //ZL Added in the 10* to scale the output

  if ( FSR_Sensors_type == 10) {
    // This to return the force instead of the Voltage
    Vo = max(0, Vo); // add the max cause cannot be negative force
  }
  else {
    if (FSR_Sensors_type == 40)
      // This to return the force instead of the Voltage
      Vo = max(0, p[0] * pow(Vo, 3) + p[1] * pow(Vo, 2) + p[2] * Vo + p[3]); // add the max cause cannot be negative force
  }

  return Vo;
}

/*Motor Current Code
 * This function reads the motor current pin and converts the voltage reading in bits to motor current.
 * For now, assume large motors, 51:1 gearbox, -7A to 7A range (0 to 3.3V)
*/
float current(const unsigned int pin) {
  //
  int value = analogRead(pin);
  float Co = 7.0 * (value - 2048.0)/2048.0;
  return Co;
}
