// Calibrate the torque sensors, FSR sensors and get the torque and FSR voltage
#include "Calibrate_and_Read_Sensors.h"

void torque_calibration()
{
  long torque_calibration_value_time = millis();
  int torq_cal_count = 0;
  left_leg->torque_calibration_value = 0;
  right_leg->torque_calibration_value = 0;

  // Creates a running total of read torque values to average after one second
  while (millis() - torque_calibration_value_time < 1000){
    left_leg->torque_calibration_value += analogRead(TORQUE_SENSOR_LEFT_ANKLE_PIN);
    right_leg->torque_calibration_value += analogRead(TORQUE_SENSOR_RIGHT_ANKLE_PIN);
    torq_cal_count++;
  }

  left_leg->torque_calibration_value *= (3.3 / 4096) / torq_cal_count;  // Averages torque over a second
  right_leg->torque_calibration_value *= (3.3 / 4096) / torq_cal_count; // Averages torque over a second
  Serial.println(left_leg->torque_calibration_value);
  Serial.println(right_leg->torque_calibration_value);
}

void calibrate_fsr_peak(double* peak_ref, int fsr_value){
  if (*peak_ref > fsr_value) {
    *peak_ref = fsr_value;
  }
}

void calibrate_fsr_leg(Leg* leg){

  leg->Curr_Toe = fsr(leg->fsr_sense_Toe);
  leg->Curr_Heel = fsr(leg->fsr_sense_Heel);
  leg->Curr_Combined = leg->Curr_Toe + leg->Curr_Heel;
  calibrate_fsr_peak(&leg->fsr_Toe_peak_ref, leg->Curr_Toe);
  calibrate_fsr_peak(&leg->fsr_Heel_peak_ref, leg->Curr_Heel);
  calibrate_fsr_peak(&leg->fsr_Combined_peak_ref, leg->Curr_Combined);
}

void FSR_calibration()
{
  if (FSR_FIRST_Cycle) {
    FSR_FIRST_Cycle = 0;

    fsrCalibrationStartTime = millis();
    Serial.println("First time");
    right_leg->Curr_Combined = 0;
    left_leg->Curr_Combined = 0;

    right_leg->fsr_Combined_peak_ref = 0;
    left_leg->fsr_Combined_peak_ref = 0;
    left_leg->fsr_Toe_peak_ref = 0;
    right_leg->fsr_Toe_peak_ref = 0;
    left_leg->fsr_Heel_peak_ref = 0;
    right_leg->fsr_Heel_peak_ref = 0;
  }


  if (millis() - fsrCalibrationStartTime < 5000)
  {
    calibrate_fsr_leg(left_leg);
    calibrate_fsr_leg(right_leg);
  }
  else {

    FSR_FIRST_Cycle = 1;
    FSR_CAL_FLAG = 0;
    if (FLAG_TWO_TOE_SENSORS) {
      Serial.println(left_leg->fsr_Combined_peak_ref);
      Serial.println(right_leg->fsr_Combined_peak_ref);
      Serial.println(" ");
    } else {
      Serial.println(left_leg->fsr_Toe_peak_ref);
      Serial.println(left_leg->fsr_Heel_peak_ref);
      Serial.println(" ");
      Serial.println(right_leg->fsr_Toe_peak_ref);
      Serial.println(right_leg->fsr_Heel_peak_ref);
      Serial.println(" ");
    }
  }
}

double get_torq(Leg* leg) {
  double Torq = 56.5 / (2.1) * (analogRead(leg->torque_sensor_ankle_pin) * (3.3 / 4096) - leg->torque_calibration_value);
  return -Torq;             //neg is here for right leg, returns the torque value of the right leg (Newton-Meters)
}

double get_LL_torq() {
  return get_torq(left_leg);
}

double get_RL_torq() {
  return get_torq(right_leg);
}

/*FSR Code
  This code is very basic but is kept as an outside function for clarity.
  The FSR readings are used to control state based actions based on the part of the gait cycle the patient is in.
*/
double fsr(const unsigned int pin) {
  //using voltage divider: 3.3 V -- >FSR<-- Vo -- >R< (1000) -- ground
  //Vo from analog read: 3.3* analog read/ max read (4096) || I = Vo/R || FSR resistance = (3.3V - Vo)/I
  double Vo = 10 * 3.3 * analogRead(pin) / 4096; //ZL Added in the 10* to scale the output

  if ( FSR_Sensors_type == 10) {
    // This to return the force instead of the Voltage
    Vo = max(0, Vo); // add the max cause cannot be negative force
  } else if (FSR_Sensors_type == 40){
    // This to return the force instead of the Voltage
    Vo = max(0, p[0] * pow(Vo, 3) + p[1] * pow(Vo, 2) + p[2] * Vo + p[3]); // add the max cause cannot be negative force
  }

  return Vo;
}
