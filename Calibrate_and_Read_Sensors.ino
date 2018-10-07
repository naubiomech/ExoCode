// Calibrate the torque sensors, FSR sensors and get the torque and FSR voltage
#include "Calibrate_and_Read_Sensors.h"
#include "Utils.h"

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
  updateMax(peak_ref, fsr_value);
}

void calibrate_fsr_leg(Leg* leg){

  double Curr_Toe = fsr(leg->fsr_sense_Toe);
  double Curr_Heel = fsr(leg->fsr_sense_Heel);
  double Curr_Combined = leg->Curr_Toe + leg->Curr_Heel;
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
