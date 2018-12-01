#ifndef POT_HEADER
#define POT_HEADER
#include "Port.hpp"

class Pot{
private:
  InputPort* port;
  double angle;
public:
  Pot(InputPort* port);
  ~Pot();
  void measure();
  double getAngle();
};

#endif
