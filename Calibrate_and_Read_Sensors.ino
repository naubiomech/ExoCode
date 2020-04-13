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
  double motor_speed = MaxSpeed * (analogRead(pin) - 2048.0)/2048.0;
  double predicted_ankle_speed = motor_speed * (1/GearRatio) * (1/PulleyRatio);
  return predicted_ankle_speed;
}

/* Read the analog output from the hall sensor at the ankle, convert to degrees, and calculate actual ankle velocity.
 *  Likely needs a calibration but for now just read and convert
 */
double ankle_angle(const unsigned int pin){
//  float a1 = -1.4611;
//  float a2 = 0.0561;
//  float a3 = 0.1440;
//  float a4 = 0.5527;
//  float a5 = 1.2733;

//  float b11 = -4.0755;
//  float b12 = -4.6774;
//  float b13 = 0;
//  float b14 = 50;
//
//  float b21 = 5.78674*pow(10,4);
//  float b22 = -7.8396*pow(10,3);
//  float b23 = -42.3369;
//  float b24 = 25;
//
//  float b31 = -190.4366;
//  float b32 = 178.7583;
//  float b33 = -77.9556;
//  float b34 = 0;
//
//  float b41 = 7.1204;
//  float b42 = 3.8118;
//  float b43 = -27.2594;
//  float b44 = -15;

  float a1 = -0.3108;
  float a2 = 0.0719;
  float a3 = 0.4338;
  float a4 = 0.4858;
  float a5 = 0.9448;
  float a6 = 1.2672;

  float b11 = 37.5394;
  float b12 = -53.4752;
  float b13 = -11.1617;
  float b14 = 50;

  float b21 = -432.3295;
  float b22 = 102.1936;
  float b23 = -35.5980;
  float b24 = 40;

  float b31 = 4.8188*pow(10,5);
  float b32 = -3.7323*pow(10,4);
  float b33 = -131.5533;
  float b34 = 20;

  float b41 = -269.8990;
  float b42 = 264.5067;
  float b43 = -108.1225;
  float b44 = -20;

  float b51 = 3.0111;
  float b52 = 14.1395;
  float b53 = -35.8835;
  float b54 = -40;
  
  float hall_voltage = 3.3*((analogRead(pin)-2048.0)/2048.0);
  double func;
  if (hall_voltage>=a1 && hall_voltage<a2) {
    func = b11 * pow((hall_voltage - a1),3) + b12*pow((hall_voltage - a1),2) + b13*(hall_voltage - a1) + b14;
  } else if (hall_voltage>=a2 && hall_voltage<a3) {
    func = b21 * pow((hall_voltage - a2),3) + b22*pow((hall_voltage - a2),2) + b23*(hall_voltage - a2) + b24;
  } else if (hall_voltage>=a3 && hall_voltage<a4) {
    func = b31 * pow((hall_voltage - a3),3) + b32*pow((hall_voltage - a3),2) + b33*(hall_voltage - a3) + b34;
  } else if (hall_voltage>=a4 && hall_voltage<a5) {
    func = b41 * pow((hall_voltage - a4),3) + b42*pow((hall_voltage - a4),2) + b43*(hall_voltage - a4) + b44;
  } else if (hall_voltage>=a5 && hall_voltage<a6) {
    func = b51 * pow((hall_voltage - a5),3) + b52*pow((hall_voltage - a5),2) + b53*(hall_voltage - a5) + b54;
  }
    return hall_voltage;
}

