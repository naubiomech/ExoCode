#ifndef PINS_HEADER
#define PINS_HEADER
#include <i2c_t3.h>

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

class IMUPins{
public:
  i2c_pins imu_slot;
  int address;
};

class FSRPins{
public:
  int fsr_pin;
};

class LegPins{
public:
  LegPins(int jointAmount, int fsrAmount, int imuAmount);
  ~LegPins();

  JointPins* joint_pins;
  int joint_count;

  FSRPins* fsr_pins;
  int fsr_count;

  IMUPins* imu_pins;
  int imu_count;
};

class ExoPins{
public:
  ExoPins(int motorsPerLeg, int fsrsPerLeg, int imuAmount);
  ~ExoPins();
  LegPins* left_leg;
  LegPins* right_leg;
  int bluetooth_rx;
  int bluetooth_tx;
  int motor_enable;
  int led;
};

#endif
