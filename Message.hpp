#ifndef EXO_MESSAGE_HEADER
#define EXO_MESSAGE_HEADER
#include <cstddef>
#include "Report.hpp"

class MotorMessage{
public:
  double* setpoint;
};

class TorqueSensorMessage{
public:
};

class JointMessage{
public:
  ~JointMessage();

  MotorMessage* motor_message = NULL;
  TorqueSensorMessage* torque_sensor_message = NULL;
};

class FSRMessage{
public:
};

class IMUMessage{
public:
};

class LegMessage{
public:
  LegMessage(LegReport* report);
  ~LegMessage();
  FSRMessage** fsr_messages;
  JointMessage** joint_messages;
  IMUMessage** imu_messages;
  int fsr_message_count;
  int joint_message_count;
  int imu_message_count;
};

class ExoMessage{
public:
  ~ExoMessage();
  LegMessage* left_leg = NULL;
  LegMessage* right_leg = NULL;
};

LegMessage* prepareMotorMessage(LegReport* report, int joint_id);
LegMessage* prepareTorqueSensorMessage(LegReport* report, int joint_id);
LegMessage* prepareFSRMessage(LegReport* report, int fsr_id);
LegMessage* prepareIMUMessage(LegReport* report, int imu_id);

#endif
