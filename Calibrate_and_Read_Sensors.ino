// Calibrate the torque sensors, FSR sensors and get the torque and FSR voltage

void torque_calibration()
{
  noInterrupts(); //Disable timer interrupts for the duration of this function
  long torque_calibration_value_time = millis();
  int torq_cal_count = 0;
  left_leg->torque_calibration_value = 0;
  right_leg->torque_calibration_value = 0;

  //left_leg->angle_zero = 0;
  //right_leg->angle_zero = 0;
  
  while (torq_cal_count < 10000) {  //(millis() - torque_calibration_value_time < 1000)  { //Calibrates the LL for a total time of 1 second,    (torq_cal_count < 10000) {
    left_leg->torque_calibration_value += analogRead(TORQUE_SENSOR_LEFT_ANKLE_PIN) * (3.3 / 4096);                                        //Sums the torque read in and sums it with all previous red values
    right_leg->torque_calibration_value += analogRead(TORQUE_SENSOR_RIGHT_ANKLE_PIN) * (3.3 / 4096);
    
    //left_leg->angle_zero += (analogRead(left_leg->ankle_angle_pin)-2048.0)*3.3/4096.0;
    //right_leg->angle_zero += (analogRead(right_leg->ankle_angle_pin) - 2048.0)*3.3/4096.0;
    
    torq_cal_count ++;                                                         //Increments count  
  }
  left_leg->torque_calibration_value = left_leg->torque_calibration_value / torq_cal_count;                       // Averages torque over a second
  right_leg->torque_calibration_value = right_leg->torque_calibration_value / torq_cal_count;                       // Averages torque over a second

  //left_leg->angle_zero = left_leg->angle_zero/torq_cal_count;
  //right_leg->angle_zero = right_leg->angle_zero/torq_cal_count;
  
  interrupts(); //Re-enable interrupts
}



void FSR_calibration()
{


  if (FSR_FIRST_Cycle) {
    FSR_FIRST_Cycle = 0;

    startTime = millis();
    //    //Serial.println("First time");
    right_leg->Curr_Combined = 0;
    left_leg->Curr_Combined = 0;

    right_leg->fsr_Combined_peak_ref = 0;
    left_leg->fsr_Combined_peak_ref = 0;
    
    left_leg->fsr_Toe_peak_ref = 0;
    right_leg->fsr_Toe_peak_ref = 0;

    left_leg->fsr_Toe_trough_ref = 1000;
    right_leg->fsr_Toe_trough_ref = 1000;
    
    left_leg->fsr_Heel_peak_ref = 0;
    right_leg->fsr_Heel_peak_ref = 0;

    //Automatically take the PJMC basline during FSR calibration
    left_leg->FSR_baseline_FLAG = 1;
    right_leg->FSR_baseline_FLAG = 1;
    left_leg->p_steps->count_plant_base = 0;
    right_leg->p_steps->count_plant_base = 0;
    right_leg->p_steps->flag_start_plant = false;
    left_leg->p_steps->flag_start_plant = false;
  }


  if (millis() - startTime < 5000)
  {
    left_leg->Curr_Toe = fsr(left_leg->fsr_sense_Toe);
    right_leg->Curr_Toe = fsr(right_leg->fsr_sense_Toe);

    left_leg->Curr_Heel = fsr(left_leg->fsr_sense_Heel);
    right_leg->Curr_Heel = fsr(right_leg->fsr_sense_Heel);

    left_leg->Curr_Combined = left_leg->Curr_Toe;// + left_leg->Curr_Heel;
    right_leg->Curr_Combined = right_leg->Curr_Toe;// + right_leg->Curr_Heel;

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

    if (left_leg->Curr_Toe < left_leg->fsr_Toe_trough_ref)
    {
      left_leg->fsr_Toe_trough_ref = left_leg->Curr_Toe;
    }

    if (right_leg->Curr_Toe < right_leg->fsr_Toe_trough_ref)
    {
      right_leg->fsr_Toe_trough_ref = right_leg->Curr_Toe;
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

  
  
  } else {

    FSR_FIRST_Cycle = 1;
    FSR_CAL_FLAG = 0;
  }
}

double get_torq(Leg* leg) {
 // double Torq = 56.5 / (2.1) * (analogRead(leg->torque_sensor_ankle_pin) * (3.3 / 4096) - leg->torque_calibration_value); //  For the TRT-500 Torque Sensor
  double Torq = ((analogRead(leg->torque_sensor_ankle_pin) * (3.3/4096.0)) - leg->torque_calibration_value)*42.500; // For the custom anchor sensor
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
      Vo = p[0] * Vo*Vo*Vo + p[1] * Vo*Vo + p[2] * Vo + p[3];
      Vo = (Vo>0.2) ? Vo: 0; //If the measured FSR value is not greater than 0.2, set it to zero. Prevents erroneous state changes during benchtop tests
  }

  return Vo;
}

/*Motor Current Code
 * This function reads the motor current pin and converts the voltage reading in bits to motor current.
*/
double current(const unsigned int pin) {
  double val = analogRead(pin)*3.3/4096.0;
  double Co = map(val,0.0,3.3,-NomCurrent,NomCurrent); //Nominal current needs to be set in ESCON, 7.58
  return Co;
}

/* This code allows us to read the analog output from the motor drivers and gives us the expected speed about the ankle.
Torque Constant (200W) : 700 rpm/V
Gear Ratio (32HP, 4-8Nm) : 17576/343 
Large Exo Pulley Ratio: 74/10.3
*/
double motor_speed(const unsigned int pin){
  double val = analogRead(pin)*3.3/4096.0;
  double motorSpeed = map(val,0.0,3.3,-MaxSpeed,MaxSpeed);
  double predicted_ankle_speed = motorSpeed * (1/GearRatio) * (1/PulleyRatio);
  return motorSpeed;
}

/* Read the analog output from the hall sensor at the ankle, convert to degrees, and calculate actual ankle velocity.
 *  Likely needs a calibration but for now just read and convert
 */
double ankle_angle(Leg* leg){

// Sensor Reading and Normalization
  double zero = leg->angle_zero;

  double hall_voltage = 3.3*((analogRead(leg->ankle_angle_pin)-2048.0)/4096.0) - zero; //Offset by zero 
  double rawAngle = -68.873*hall_voltage; //Raw voltage regression for comparisons

  return rawAngle;
}
