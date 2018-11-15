#ifndef EXO_REPORT_HEADER
#define EXO_REPORT_HEADER
#include "Phase.hpp"

class MotorReport{
public:
  double pid_setpoint;
};

class TorqueSensorReport{
public:
  double measuredTorque;
};

class JointReport{
public:
  ~JointReport();

  MotorReport* motor_report;
  TorqueSensorReport* torque_sensor_report;
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
  Phase phase;
};

class ExoReport{
public:
  ~ExoReport();
  LegReport* left_leg;
  LegReport* right_leg;
};


#endif
