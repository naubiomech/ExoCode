#ifndef TORQUE_SENSOR_HEADER
#define TORQUE_SENSOR_HEADER

#include "Report.hpp"
#include "Pins.hpp"
#include "Utils.hpp"

class TorqueSensor{
private:
  unsigned int torque_sensor_pin;
  MovingAverage* torque_averager;
  RunningAverage* torque_calibration_average;
  double torque_calibration_value = 0;
  int torque_address;

  double measureRawTorque();
  double measureRawCalibratedTorque();
  void fillLocalReport(TorqueSensorReport* report);
public:
  double getTorque();
  TorqueSensor(TorqueSensorPins* sensor_pins);
  void measureTorque();
  void startTorqueCalibration();
  void updateTorqueCalibration();
  void endTorqueCalibration();
  TorqueSensorReport* generateReport();
  void fillReport(TorqueSensorReport* report);
};

#endif
