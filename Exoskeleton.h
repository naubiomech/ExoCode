#ifndef EXOSKELETON_HEADER
#define EXOSKELETON_HEADER
#include "Leg.h"

class Exoskeleton{
private:
public:
  Leg* left_leg;
  Leg* right_leg;

  Exoskeleton();
  void measureSensors();
  bool checkMotorErrors();
};

#endif
