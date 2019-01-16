#include "Pot.hpp"

Pot::Pot(InputPort* port){
  this->port = port;
}

Pot::~Pot(){
  delete port;
}

void Pot::measure(){
  angle = port->read() * 180.0;
}

double Pot::getAngle(){
  return angle;
}

void Pot::fillReport(PotReport* report){
  report->angle = this->getAngle();
}
