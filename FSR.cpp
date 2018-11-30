#include "Arduino.hpp"
#include "FSR.hpp"
#include "Parameters.hpp"
#include "Utils.hpp"
#include "Report.hpp"

FSR::FSR(InputPort* port){
  this->port = port;
}

void FSR::measureForce(){
  double force = port->read();

  if ( FSR_Sensors_type == 10) {
    force = max(0, force);
  } else if (FSR_Sensors_type == 40){
    force = max(0, p[0] * pow(force, 3) + p[1] * pow(force, 2) + p[2] * force + p[3]);
  }
  updateForce(force);
}

void FSR::updateForce(double force){
  max_force->update(force);
  force /= calibration_peak;
}

void FSR::calibrate(){
  calibration_peak = peak_average->getAverage();
}

void FSR::resetMaxes(){
  peak_average->update(max_force->getMax());
  max_force->reset();
}

double FSR::getForce(){
  return force;
}

FSRGroup::FSRGroup(LinkedList<FSR*> fsrs){
  activation_threshold = new Threshold(0, fsr_percent_thresh, state_counter_th);
  this->fsr_count = fsrs.size();
  this->fsrs = fsrs;
}

bool FSRGroup::isActivated(){
  return is_activated;
}

void FSRGroup::measureForce(){
  double average = 0;
  for (int i = 0; i < fsr_count; i++){
    fsrs[i]->measureForce();
    average += fsrs[i]->getForce();
  }
  force = average / fsr_count;
  is_activated = activation_threshold->getState(force);
}

void FSRGroup::calibrate(){
  for (int i = 0; i < fsr_count; i++){
    fsrs[i]->calibrate();
  }
}

void FSRGroup::setPercentageThreshold(double percent){
  fsr_percent_thresh = percent;
}

double FSRGroup::getThreshold(){
  return fsr_percent_thresh;
}

void FSRGroup::resetMaxes(){
  for(unsigned int i = 0; i < fsrs.size(); i++){
    fsrs[i]->resetMaxes();
  }
}

double FSRGroup::getPercentage(){
  return force;
}

double FSRGroup::getMaxPercentage(){
  return 1;
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
  report->measuredForce = getPercentage();
}
