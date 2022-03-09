#include "BatteryMonitor.h"

processed_bat_t BatteryMonitor::handle(int new_value)
{
  processed_bat_t msg;
  //Scale
  int scaled_value = 1.25*(new_value) - 25;
  msg.scaled_val = scaled_value;
  //Check 
  msg.too_low = _check(new_value);
  return msg;
}

bool BatteryMonitor::_check(int new_value)
{
  return (new_value <= this->_shutdown_value);
}
