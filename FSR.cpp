#include <Arduino.h>
#include "FSR.hpp"
#include "Parameters.hpp"
#include "Utils.hpp"
#include "Report.hpp"

FSR::FSR(InputPort* port){
  this->port = port;
}

void FSR::measureForce(){
  double Vo = 10 * 3.3 * port->read() / 4096;

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

FSRGroup::FSRGroup(std::vector<FSR*> fsrs){
  this->fsr_count = fsrs.size();
  this->fsrs = fsrs;
}

void FSRGroup::measureForce(){
  double average = 0;
  for (int i = 0; i < fsr_count; i++){
    fsrs[i]->measureForce();
    average += fsrs[i]->getForce();
  }
  force = average / fsr_count;
  max_fsr_voltage->update(force);
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
