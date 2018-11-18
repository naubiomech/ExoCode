#include <Arduino.h>
#include "FSR.hpp"
#include "Parameters.hpp"
#include "Utils.hpp"
#include "Report.hpp"

FSR::FSR(int pin){
  this->pin = pin;
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

FSRGroup::FSRGroup(FSRGroupPins* fsr_group_pins){
  this->fsr_count = fsr_group_pins->fsr_count;
  this->fsrs = new FSR*[fsr_count];
  for (int i = 0; i < fsr_count; i++){
    this->fsrs[i] = new FSR(fsr_group_pins->fsr_pins[i]);
  }
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
  calibration_peak = peak_average->getAverage();
}

void FSRGroup::setPercentageThreshold(double percent){
  fsr_percent_thresh = percent;
}

double FSRGroup::getThreshold(){
  return calibration_peak * fsr_percent_thresh;
}

void FSRGroup::resetMaxes(){
  double max_voltage = this->max_fsr_voltage->getMax();
  this->peak_average->update(max_voltage);

  this->max_fsr_voltage->reset();
}

void FSRGroup::updateMaxes(){
  this->max_fsr_voltage->update(this->getForce());
}

double FSRGroup::getPercentage(){
  return fsr_percentage;
}

double FSRGroup::getMaxPercentage(){
  double max_fsr_percentage = fabs(max_fsr_voltage->getMax() / calibration_peak);
  return max_fsr_percentage;
}

FSRReport* FSRGroup::generateReport(){
  FSRReport* report = new FSRReport();
  fillLocalReport(report);
  return report;
}

void FSRGroup::fillReport(FSRReport* report){
  fillLocalReport(report);
}

void FSRGroup::fillLocalReport(FSRReport* report){
  report->threshold = getThreshold();
  report->measuredForce = getForce();
}
