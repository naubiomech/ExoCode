#include "BatteryMonitor.h"

bool BatteryMonitor::handle(int& new_value)
{
  //Scale
  float m = (100/(100-this->_new_zero));
  float b = (1 - m) * 100;
  int scaled_value = m*(new_value) + b;
  new_value = scaled_value;
  //Check 
  return this->_check(new_value);
}

bool BatteryMonitor::_check(int new_value)
{
  return (new_value <= this->_shutdown_value);
}
