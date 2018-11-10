#ifndef PINS_HEADER
#define PINS_HEADER

class MotorPins{
public:
  int err;
  int torque;
  int motor;
};

class FSRPins{
public:
  int fsr_pin;
};

class LegPins{
public:
  LegPins(int motorsAmount, int fsrAmount);
  ~LegPins();
  MotorPins* motor_pins;
  int motor_count;
  FSRPins* fsr_pins;
  int fsr_count;
};

class ExoPins{
public:
  ExoPins(int motorsPerLeg, int fsrsPerLeg);
  ~ExoPins();
  LegPins* left_leg;
  LegPins* right_leg;
  int bluetooth_rx;
  int bluetooth_tx;
  int motor_enable;
  int led;
};

#endif
