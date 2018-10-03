#include "FSR.h"

FSR::FSR(int pin){
  this.pin = pin;
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

FSRGroup::FSRGroup(int* fsr_pins, int fsr_count){
  this.fsrs = new[](fsr_count * sizeof(*this.fsrs));
  for (int i = 0; i < fsr_count; i++){
    this.fsrs[i] = fsrs[i];
  }
  this.fsr_count = fsr_count;
}

void FSRGroup::measureForce(){
  double average = 0;
  for (int i = 0; i < fsr_count; i++){
    average += fsrs[i]->measureForce();
  }
  force = average / fsr_count;
}

double FSRGroup::getForce(){
  return force;
}
