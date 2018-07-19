// Calibrate the torque sensors, FSR sensors and get the torque and FSR voltage

void torque_calibration()
{
  long tcal_time = millis();
  int torq_cal_count = 0;
  Tcal_LL = 0;
  Tcal_RL = 0;
  while (millis() - tcal_time < 1000)
  { //Calibrates the LL for a total time of 1 second,
    Tcal_LL += analogRead(A19);                                        //Sums the torque read in and sums it with all previous red values
    Tcal_RL += analogRead(A18);
    torq_cal_count ++;                                                         //Increments count
  }
  Tcal_LL = (Tcal_LL / torq_cal_count) * (3.3 / 4096);                         // Averages torque over a second
  Tcal_RL = (Tcal_RL / torq_cal_count) * (3.3 / 4096);                         // Averages torque over a second
}

void FSR_calibration()
{
  if (FSR_FIRST_Cycle) {
    FSR_FIRST_Cycle = 0;

    store_KF_LL = KF_LL;
    store_KF_RL = KF_RL;
    KF_LL = 0;
    KF_RL = 0;

    //    double Curr_Left_Toe;
    //    double Curr_Left_Heel;
    //    double Curr_Right_Toe;
    //    double Curr_Right_Heel;

    //  fsr_cal_Short = 0;
    //    fsr_cal_Long = 0;
    startTime = millis();
    Serial.println("First time");
    Curr_Left_Toe = 0;
    Curr_Left_Heel = 0;
    Curr_Right_Toe = 0;
    Curr_Right_Heel = 0;
    fsr_Left_Toe_thresh = 0;
    fsr_Left_Heel_thresh = 0;
    fsr_Right_Toe_thresh = 0;
    fsr_Right_Heel_thresh = 0;
  }


  if (millis() - startTime < 5000)
  {
    //      fsrLongCurrent = fsr(fsr_sense_Long);
    //      fsrShortCurrent = fsr(fsr_sense_Short);

    Curr_Left_Toe = fsr(fsr_sense_Left_Toe);
    Curr_Left_Heel = fsr(fsr_sense_Left_Heel);
    Curr_Right_Toe = fsr(fsr_sense_Right_Toe);
    Curr_Right_Heel = fsr(fsr_sense_Right_Heel);

    if (Curr_Left_Toe > fsr_Left_Toe_thresh)
    {
      fsr_Left_Toe_thresh = Curr_Left_Toe;
    }

    if ( Curr_Left_Heel > fsr_Left_Heel_thresh)
    {
      fsr_Left_Heel_thresh = Curr_Left_Heel;
    }

    if (Curr_Right_Toe > fsr_Right_Toe_thresh)
    {
      fsr_Right_Toe_thresh = Curr_Right_Toe;
    }

    if ( Curr_Right_Heel > fsr_Right_Heel_thresh)
    {
      fsr_Left_Heel_thresh = Curr_Left_Heel;
    }
  }
  else {
    Serial.println("else <5000");
    //Update structure with the threshold
    L_p_steps->voltage_ref = fsr_Left_Toe_thresh;
    R_p_steps->voltage_ref = fsr_Right_Toe_thresh;

    write_FSR_values(address_FSR_LL, fsr_Left_Heel_thresh);
    write_FSR_values((address_FSR_LL + sizeof(double) + sizeof(char)), fsr_Left_Toe_thresh);
    write_FSR_values(address_FSR_RL, fsr_Right_Heel_thresh);
    write_FSR_values((address_FSR_RL + sizeof(double) + sizeof(char)), fsr_Right_Toe_thresh);

    //      L_p_steps->voltage_ref = fsr_Left_Toe_thresh;
    //      R_p_steps->voltage_ref = fsr_Right_Toe_thresh;

    KF_LL = store_KF_LL;
    KF_RL = store_KF_RL;
    FSR_FIRST_Cycle = 1;
    FSR_CAL_FLAG = 0;

    Serial.println(fsr_Right_Toe_thresh);
    Serial.println(fsr_Left_Toe_thresh);
    Serial.println(" ");
  }
}

double get_LL_torq()
{ //flexion is positive 8.10.16, gets the torque of the right leg
  double Torq = 56.5 / (2.1) * (analogRead(A19) * (3.3 / 4096) - Tcal_LL);
  return -Torq;             //neg is here for right leg, returns the torque value of the right leg (Newton-Meters)
}

double get_RL_torq()
{ //flexion is positive 8.10.16, gets the torque of the right leg
  double Torq = 56.5 / (2.1) * (analogRead(A18) * (3.3 / 4096) - Tcal_RL);
  return -Torq;             //neg is here for right leg, returns the torque value of the right leg (Newton-Meters)
}


/*FSR Code
   This code is very basic but is kept as an outside function for clarity. The FSR readings are used to control state based actions based on the part of the gait cycle the patient
   is in.
*/
double fsr(const unsigned int pin) {
  double Vo;
  analogRead(pin);
  //using voltage divider: 3.3 V -- >FSR<-- Vo -- >R< (1000) -- ground
  //Vo from analog read: 3.3* analog read/ max read (4096) || I = Vo/R || FSR resistance = (3.3V - Vo)/I
  Vo = 10 * 3.3 * analogRead(pin) / 4096; //ZL Added in the 10* to scale the output

  //return (3.3 - Vo)*1000/Vo; //this is FSR resistance value
  return Vo;
}
