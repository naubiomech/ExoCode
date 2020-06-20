// Calibrate the torque sensors, FSR sensors and get the torque and FSR voltage

void torque_calibration()
{
  long torque_calibration_value_time = millis();
  int torq_cal_count = 0;
  left_leg->torque_calibration_value = 0;
  right_leg->torque_calibration_value = 0;

  while (millis() - torque_calibration_value_time < 2000)
  { //Calibrates the LL for a total time of 2 second,
    left_leg->torque_calibration_value += analogRead(TORQUE_SENSOR_LEFT_ANKLE_PIN) * (3.3 / 4096);                                        //Sums the torque read in and sums it with all previous red values
    right_leg->torque_calibration_value += analogRead(TORQUE_SENSOR_RIGHT_ANKLE_PIN) * (3.3 / 4096);
    torq_cal_count ++;                                                         //Increments count
  }
  left_leg->torque_calibration_value = left_leg->torque_calibration_value / torq_cal_count;                       // Averages torque over a second
  right_leg->torque_calibration_value = right_leg->torque_calibration_value / torq_cal_count;                       // Averages torque over a second
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
 // double Torq = 56.5 / (2.1) * (analogRead(leg->torque_sensor_ankle_pin) * (3.3 / 4096) - leg->torque_calibration_value); //Cylindrical torque sensor
  //double Torq = ((analogRead(leg->torque_sensor_ankle_pin) * (3.3/4096.0)) - leg->torque_calibration_value)*52.948;  //Anchor torque sensor
  double Torq = ((analogRead(leg->torque_sensor_ankle_pin) * (3.3/4096.0)) - leg->torque_calibration_value)*49.39; //Anchor torque sensor v2
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
*/
double current(const unsigned int pin) {
  //
  int value = analogRead(pin);
  double Co = NomCurrent * (value - 2048.0)/2048.0; //Nominal current needs to be set in ESCON, 7.58
  return Co;
}

/* This code allows us to read the analog output from the motor drivers and gives us the expected torque about the ankle.
Torque Constant (200W) : 13.6 mNm/A
Gear Ratio (32HP, 4-8Nm) : 17576/343 
Large Exo Pulley Ratio: 74/10.3
*/
double expected_ankle_torq(const unsigned int pin){
  double motor_current = current(pin);
  double ankle_torq = motor_current * TrqConstant * GearRatio * PulleyRatio;
  return ankle_torq;
}

/* This code allows us to read the analog output from the motor drivers and gives us the expected speed about the ankle.
Torque Constant (200W) : 700 rpm/V
Gear Ratio (32HP, 4-8Nm) : 17576/343 
Large Exo Pulley Ratio: 74/10.3
*/
double motor_ankle_speed(const unsigned int pin){
  double motor_speed = map(analogRead(pin),0,4096,-MaxSpeed,MaxSpeed);
  //double motor_speed = MaxSpeed*3.3*(analogRead(pin)-2048)/2048;
  double predicted_ankle_speed = motor_speed * (1/GearRatio) * (1/PulleyRatio);
  return motor_speed;
}

/* Read the analog output from the hall sensor at the ankle, convert to degrees, and calculate actual ankle velocity.
 *  Likely needs a calibration but for now just read and convert
 */
double ankle_angle(Leg* leg){

// Sensor Reading and Normalization
  double zeroL = 0.5505;
  double zeroR = -0.0983;
  double maxPFXL = -0.6475 - zeroL;
  double maxDFXL = 0.959 - zeroL;
  double maxPFXR = -1.5284;
  double maxDFXR = 1.8640;
  double zero;
  double maxPFX;
  double maxDFX;
  double angle;

if (leg->whos == 'L') {
  zero = zeroL;
  maxPFX = maxPFXL;
  maxDFX = maxDFXL;
} else {
  zero = zeroR;
  maxPFX = maxPFXR;
  maxDFX = maxDFXR; 
}

float hall_voltage = 3.3*((analogRead(leg->ankle_angle_pin)-2048.0)/2048.0) - zero; //Offset by zero 
if (hall_voltage > 0) {
  hall_voltage = hall_voltage/abs(maxDFX); //Positive voltage is dorsiflexion
} else if (hall_voltage < 0) {
  hall_voltage = hall_voltage/abs(maxPFX); //Negative voltage is plantarflexion
} else {
  hall_voltage = 0;
}

  angle = -1.675 - 0.5947*asin(hall_voltage)*180/PI; //Calculate the angle from the normalized and transformed voltag
  return angle;
}
