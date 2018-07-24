// Calibrate the torque sensors, FSR sensors and get the torque and FSR voltage

void torque_calibration()
{
  long tcal_time = millis();
  int torq_cal_count = 0;
  Tcal_LL = 0;
  Tcal_RL = 0;
  double Tcal_LL_val, Tcal_RL_val;
  while (millis() - tcal_time < 1000)
  { //Calibrates the LL for a total time of 1 second,
    Tcal_LL_val += analogRead(TORQUE_SENSOR_LEFT_ANKLE_PIN) * (3.3 / 4096);                                        //Sums the torque read in and sums it with all previous red values
    Tcal_RL_val += analogRead(TORQUE_SENSOR_RIGHT_ANKLE_PIN) * (3.3 / 4096);
    torq_cal_count ++;                                                         //Increments count
  }
  Tcal_LL = Tcal_LL_val / torq_cal_count;                       // Averages torque over a second
  Tcal_RL = Tcal_RL_val / torq_cal_count;                       // Averages torque over a second
  Serial.println(Tcal_LL);
  Serial.println(Tcal_RL);
}



void FSR_calibration()
{


  if (FSR_FIRST_Cycle) {
    FSR_FIRST_Cycle = 0;

    startTime = millis();
    Serial.println("First time");
    Curr_Combined_Right = 0;
    Curr_Combined_Left = 0;

    fsr_Right_Combined_peak_ref = 0;
    fsr_Left_Combined_peak_ref = 0;
  }


  if (millis() - startTime < 5000)
  {

    Curr_Combined_Right = fsr(fsr_sense_Right_Toe) + fsr(fsr_sense_Right_Heel);
    Curr_Combined_Left = fsr(fsr_sense_Left_Toe) + fsr(fsr_sense_Left_Heel);

    if (Curr_Combined_Left > fsr_Left_Combined_peak_ref)
    {
      fsr_Left_Combined_peak_ref = Curr_Combined_Left;
    }

    if (Curr_Combined_Right > fsr_Right_Combined_peak_ref)
    {
      fsr_Right_Combined_peak_ref = Curr_Combined_Right;
    }

  }
  else {
    L_p_steps->voltage_peak_ref = fsr_Left_Combined_peak_ref;
    R_p_steps->voltage_peak_ref = fsr_Right_Combined_peak_ref;

    // What I need to comment out

    write_FSR_values(address_FSR_LL, fsr_Left_Combined_peak_ref / 2);
    write_FSR_values((address_FSR_LL + sizeof(double) + sizeof(char)), fsr_Left_Combined_peak_ref / 2);
    write_FSR_values(address_FSR_RL, fsr_Right_Combined_peak_ref / 2);
    write_FSR_values((address_FSR_RL + sizeof(double) + sizeof(char)), fsr_Right_Combined_peak_ref / 2);

    FSR_FIRST_Cycle = 1;
    FSR_CAL_FLAG = 0;

    Serial.println(fsr_Right_Combined_peak_ref);
    Serial.println(fsr_Left_Combined_peak_ref);
    Serial.println(" ");
  }
}

double get_LL_torq()
{ //flexion is positive 8.10.16, gets the torque of the right leg
  double Torq = 56.5 / (2.1) * (analogRead(TORQUE_SENSOR_LEFT_ANKLE_PIN) * (3.3 / 4096) - Tcal_LL);
  return -Torq;             //neg is here for right leg, returns the torque value of the right leg (Newton-Meters)
}

double get_RL_torq()
{ //flexion is positive 8.10.16, gets the torque of the right leg
  double Torq = 56.5 / (2.1) * (analogRead(TORQUE_SENSOR_RIGHT_ANKLE_PIN) * (3.3 / 4096) - Tcal_RL);
  return -Torq;             //neg is here for right leg, returns the torque value of the right leg (Newton-Meters)
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
