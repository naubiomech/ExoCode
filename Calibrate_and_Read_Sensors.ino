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
  double zeroR = 1.2585;
  double maxPFXL = -0.6475 - zeroL;
  double maxDFXL = 0.959 - zeroL;
  double maxPFXR = 0.4375-zeroR;
  double maxDFXR = 1.5495-zeroR;
  double zero;
  double maxPFX;
  double maxDFX;

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

// Piecewise Polynomial

  float p1 = -375.90;
  float p2 = -1644.23;
  float p3 = -3451.21;
  float p4 = -3321.69;
  float p5 = -1188.87;

  float d1 = -50.23;
  float d2 = 55.48;
  float d3 = -30.296;

  double pp;
  if (hall_voltage > 0) { //If DFX
    pp = d1*hall_voltage + d2*pow(hall_voltage,2) + d3*pow(hall_voltage,3); //DFX angle in degrees
  } else if (hall_voltage < 0) { //If PFX
    pp = p1*hall_voltage + p2*pow(hall_voltage,2) + p3*pow(hall_voltage,3) + p4*pow(hall_voltage,4) + p5*pow(hall_voltage,5);
  } else {
    pp = 0;
  }
 
// Piecewise Spline

float a1;
float a2;
float a3;
float a4;
float a5;
float a6;
float a7;
float a8;

float b11;
float b12;
float b13;
float b14;

float b21;
float b22;
float b23;
float b24;

float b31;
float b32;
float b33;
float b34;

float b41;
float b42;
float b43;
float b44;

float b51;
float b52;
float b53;
float b54;

float b61;
float b62;
float b63;
float b64;

float b71;
float b72;
float b73;
float b74;

if (leg->whos == 'L') {
  a1 = -1;
  a2 = -0.822119380081830;
  a3 =-0.132447823889784;
  a4 = -0.0425534772600975;
  a5 = 0;
  a6 = 0.228572250345145;
  a7 = 0.743097100782329;
  a8 = 1;  

  b11 = 52.1591538064012;
  b21 = -53.8832744360493;
  b31 = -1939.52520551072;
  b41 = 357310.078259034;
  b51 = -758.088812877682;
  b61 = -33.5865199919828;
  b71 = 0.0460180706737880;

  b12 = -1.94798884654023;
  b22 = 42.0988601150009;
  b32 = -573.945914179372;
  b42 = -21686.3733761400;
  b52 = 414.097071446106;
  b62 = 34.5328430148431;
  b72 = -0.0589025149632422;

  b13 = -29.4126246394564;
  b23 = -25.1544596330330;
  b33 = -43.9739170299623;
  b43 = -194.182775003996;
  b53 = -98.7943847267092;
  b63 = -28.3118581521727;
  b73 = -19.4505112835239;

  b14 = 50;
  b24 = 45;
  b34 = 30;
  b44 = 20;
  b54 = 0;
  b64 = -10;
  b74 = -20;
  
} else if (leg->whos == 'R') {

  a1 = -1;
  a2 = -0.881568383616493;
  a3 = -0.213979299121880;
  a4 = -0.0869270618031333;
  a5 = 0;
  a6 = 0.302716779515782;
  a7 = 0.805641895865187;
  a8 = 1;  

  b11 = 546.765433526463;
  b21 = -59.7643607551202;
  b31 = -221.946737622983;
  b41 = 36011.6408726956;
  b51 = -287.221056358903;
  b61 = -33.3757793334474;
  b71 = 32.6884689789540;

  b12 = -39.6283286033024;
  b22 = 53.5364776994065;
  b32 = -276.465267423479;
  b42 = -4385.22188187308;
  b52 = 199.288228453309;
  b62 = 27.6480117235190;
  b72 = -21.0848161699680;

  b13 = -45.1941671968177;
  b23 = -31.5737869677671;
  b33 = -39.9995284387991;
  b43 = -120.998763136911;
  b53 = -67.0418639960560;
  b63 = -25.3466969376039;
  b73 = -22.8625132454442;

  b14 = 50;
  b24 = 45;
  b34 = 30;
  b44 = 20;
  b54 = 0;
  b64 = -10;
  b74 = -20;
  

} else {

}

  double ps;
  if (hall_voltage>=a1 && hall_voltage<a2) {
    ps = b11 * pow((hall_voltage - a1),3) + b12*pow((hall_voltage - a1),2) + b13*(hall_voltage - a1) + b14;
  } else if (hall_voltage>=a2 && hall_voltage<a3) {
    ps = b21 * pow((hall_voltage - a2),3) + b22*pow((hall_voltage - a2),2) + b23*(hall_voltage - a2) + b24;
  } else if (hall_voltage>=a3 && hall_voltage<a4) {
    ps = b31 * pow((hall_voltage - a3),3) + b32*pow((hall_voltage - a3),2) + b33*(hall_voltage - a3) + b34;
  } else if (hall_voltage>=a4 && hall_voltage<a5) {
    ps = b41 * pow((hall_voltage - a4),3) + b42*pow((hall_voltage - a4),2) + b43*(hall_voltage - a4) + b44;
  } else if (hall_voltage>=a5 && hall_voltage<a6) {
    ps = b51 * pow((hall_voltage - a5),3) + b52*pow((hall_voltage - a5),2) + b53*(hall_voltage - a5) + b54;
  } else if (hall_voltage>=a6 && hall_voltage<a7) {
    ps = b61 * pow((hall_voltage - a6),3) + b62*pow((hall_voltage - a6),2) + b63*(hall_voltage - a6) + b64;
  } else if (hall_voltage>=a7 && hall_voltage<a8) {
    ps = b71 * pow((hall_voltage - a7),3) + b72*pow((hall_voltage - a7),2) + b73*(hall_voltage - a7) + b74;
  }

  // Averaging
  double angle;
  if ((hall_voltage>1) || (hall_voltage<-1)) {
    angle = pp; // Try and predict angle beyond calibrated maximum
  } else {
    angle = ps;
  } 
  return angle;
}
