#include <Arduino.h>
#include "FSR.hpp"
#include "Parameters.hpp"
#include "Utils.hpp"
#include "Report.hpp"

FSR::FSR(int pin){
  this->pin = pin;
}

double FSR::calibrate(){
  if (!isCalibrating) {
    startCalibration();
  }

  if (millis() - fsrCalibrationStartTime < FSR_CALIBRATION_TIME_MS) {
    updateCalibration();
  } else {
    endCalibration();
  }
  return peak_force;
}

void FSR::startCalibration(){
  isCalibrating = true;
  fsrCalibrationStartTime = millis();
  peak_force = 0;
}

void FSR::updateCalibration(){
  updateMax(&peak_force, getForce());
}

void FSR::endCalibration(){
  isCalibrating = false;
}

void FSR::measureForce(){
  double Vo = 10 * 3.3 * analogRead(pin) / 4096;

  if ( FSR_Sensors_type == 10) {
    Vo = max(0, Vo);
  } else if (FSR_Sensors_type == 40){
    Vo = max(0, p[0] * pow(Vo, 3) + p[1] * pow(Vo, 2) + p[2] * Vo + p[3]);
  }

  force = Vo;
}

double FSR::getForce(){
  return force;
}

double FSR::getBalanceReference(){
  return min(1, (getForce() / peak_force));
}

FSRGroup::FSRGroup(FSRPins* fsr_pins, int fsr_count){
  this->fsrs = new FSR*[fsr_count];
  for (int i = 0; i < fsr_count; i++){
    this->fsrs[i] = new FSR(fsr_pins[i].fsr_pin);
  }
  this->fsr_count = fsr_count;
}

void FSRGroup::measureForce(){
  double average = 0;
  for (int i = 0; i < fsr_count; i++){
    fsrs[i]->measureForce();
    average += fsrs[i]->getForce();
  }
  force = average / fsr_count;
}

double FSRGroup::getForce(){
  return force;
}

void FSRGroup::calibrate(){
  double peak = 0;
  for(int i = 0; i< fsr_count;i++){
    peak += fsrs[i]->calibrate();
  }
  calibration_peak = peak;
}

double FSRGroup::getBalanceReference(){
  return fsrs[0]->getBalanceReference() - fsrs[1]->getBalanceReference();
}

void FSRGroup::setPercentageThreshold(double percent){
  fsr_percent_thresh = percent;
}

double FSRGroup::getThreshold(){
  return calibration_peak * fsr_percent_thresh;
}

void FSRGroup::resetMaxes(){
  this->max_fsr_voltage->reset();
  this->max_fsr_percentage->reset();
}

void FSRGroup::updateMaxes(){

  this->max_fsr_voltage->update(this->getForce());
  double FSRatio = fabs(this->getForce() / this->plant_peak_mean);
  this->max_fsr_percentage->update(FSRatio);
}

double FSRGroup::getPercentage(){
  return fsr_percentage;
}

double FSRGroup::getMaxPercentage(){
  return max_fsr_percentage->getMax();
}

void FSRGroup::startBaseline(){

  this->plant_peak_averager->reset();
}

void FSRGroup::updateBaseline(){
  double plant_fsr_peak = this->max_fsr_voltage->getMax();
  this->plant_peak_averager->update(plant_fsr_peak);
  this->plant_peak_mean = this->plant_peak_averager->getAverage();
}

FSRReport FSRGroup::generateReport(){
	FSRReport report = FSRReport();
	report.threshold = getThreshold();
	report.measuredForce = getForce();
	return report;
}

void FSRGroup::fillReport(FSRReport* report){
	report->threshold = getThreshold();
	report->measuredForce = getForce();
}
