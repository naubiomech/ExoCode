#ifndef PINS_HEADER
#define PINS_HEADER

class ExoPins{
  ExoPins(int motorsPerLeg, int fsrsPerLeg);
  LegPins* left_leg;
  LegPins* right_leg;
  int bluetooth_rx;
  int bluetooth_tx;
  int motor_enable;
  int led;
};

class LegPins{
  LegPins(int motorsAmount, int fsrAmount);
  MotorPins* motor_pins;
  int motor_count;
  FSRPins* fsr_pins;
  int fsr_count;
};

class MotorPins{
  int err;
  int torque;
  int motor;
};

class FSRPins{
  int fsr_pin;
};

#endif
