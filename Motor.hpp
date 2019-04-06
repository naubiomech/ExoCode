#ifndef MOTOR_HEADER
#define MOTOR_HEADER
#include "Arduino.hpp"
#include "Parameters.hpp"
#include "Utils.hpp"
#include "Shaping_Functions.hpp"
#include "Control_Algorithms.hpp"
#include "Port.hpp"
#include "Report.hpp"

class Motor{
private:
  void fillLocalReport(MotorReport* report);

  double output_sign;
  OutputPort* motor_port;
  InputPort* motor_error_port;

  bool in_error_state;
  double zero_offset;

public:
  Motor(InputPort* motor_error_port, OutputPort* motor_port, int output_sign);
  virtual ~Motor();
  virtual void measureError();
  virtual bool hasErrored();
  virtual void write(double value);
  virtual void setSign(int sign);
  virtual MotorReport* generateReport();
  virtual void fillReport(MotorReport* report);
};

#endif
