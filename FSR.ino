#include "FSR.h"
#include "Utils.h"

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
  FSR_CAL_FLAG = 0;
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

void FSRGroup::getThreshold(){
  return calibration_peak * fsr_percent_thresh;
}
