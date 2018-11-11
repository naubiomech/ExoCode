#ifndef PINS_HEADER
#define PINS_HEADER

class MotorPins{
public:
  int err;
  int motor;
};

class TorqueSensorPins{
public:
  int torque_sensor;
};

class JointPins{
public:
  MotorPins motor_pins;
	TorqueSensorPins torque_sensor_pins;
};

class FSRPins{
public:
  int fsr_pin;
};

class LegPins{
public:
  LegPins(int jointAmount, int fsrAmount);
  ~LegPins();
  JointPins* joint_pins;
  int joint_count;
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
