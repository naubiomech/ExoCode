#ifndef MOTOR_HEADER
#define MOTOR_HEADER
#include "Parameters.hpp"
#include "Utils.hpp"
#include "Shaping_Functions.hpp"
#include "Control_Algorithms.hpp"
#include "Pins.hpp"
#include "Report.hpp"
#include <PID_v2.h>

class Motor{
private:
  void fillLocalReport(MotorReport* report);

  double output_sign = 1.0;
  unsigned int motor_pin;
  unsigned int motor_error_pin;

  bool in_error_state;
  double zero_torque_reference = ZERO_TORQUE_REFERENCE_DEFAULT;

public:
  Motor(MotorPins* motor_pins);
  void measureError();
  bool hasErrored();
  void write(double value);
  void setSign(int sign);
  MotorReport* generateReport();
  void fillReport(MotorReport* report);
};

#endif
