#include "Leg.h"

class Exoskeleton{
private:
  Leg* left_leg;
  Leg* right_leg;
public:
  Exoskeleton();
  void measureSensors();
};
