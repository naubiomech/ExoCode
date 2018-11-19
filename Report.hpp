#ifndef EXO_REPORT_HEADER
#define EXO_REPORT_HEADER
#include <cstddef>
#include "States.hpp"

class MotorReport{
public:
  double error;
};

class TorqueSensorReport{
public:
  double measuredTorque;
};

class JointReport{
public:
  ~JointReport();

  double pid_setpoint;
  MotorReport* motor_report = NULL;
  TorqueSensorReport* torque_sensor_report = NULL;
};

class FSRReport{
public:
  double threshold;
  double measuredForce;
};

class IMUReport{
public:
  double orientation[3];
};

class LegReport{
public:
  LegReport(int joint_count, int fsr_count, int imu_count);
  ~LegReport();
  FSRReport** fsr_reports;
  JointReport** joint_reports;
  IMUReport** imu_reports;
  int fsr_report_count;
  int joint_report_count;
  int imu_report_count;
  int state;
  StateType phase;
};

class ExoReport{
public:
  ~ExoReport();
  LegReport* left_leg = NULL;
  LegReport* right_leg = NULL;
};


#endif
