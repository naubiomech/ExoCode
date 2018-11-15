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
  JointPins();
  ~JointPins();

  MotorPins* motor_pins;
  TorqueSensorPins* torque_sensor_pins;
};

class IMUPins{
public:
  i2c_pins imu_slot;
  int address;
};

class FSRGroupPins{
public:
  FSRGroupPins(int fsr_count);
  ~FSRGroupPins();

  int* fsr_pins;
  int fsr_count;
};

class LegPins{
private:
  int alloced_joint_count;
  int alloced_fsr_group_count;
  int alloced_imu_count;
public:
  LegPins(int joints_per_leg, int fsr_groups_per_leg, int fsrs_per_group, int imus_per_leg, int leg_sign);
  ~LegPins();

  int leg_sign = 1;

  JointPins** joint_pins;
  int joint_count;

  FSRGroupPins** fsr_groups_pins;
  int fsr_group_count;

  IMUPins** imu_pins;
  int imu_count;
};

class ExoPins{
public:
  ExoPins(int joints_per_leg, int fsr_groups_per_leg, int fsrs_per_group, int imus_per_leg);
  ~ExoPins();
  LegPins* left_leg;
  LegPins* right_leg;
  int bluetooth_rx;
  int bluetooth_tx;
  int motor_enable;
  int led;
};

#endif
