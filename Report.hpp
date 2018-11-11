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
	MotorReport motor_report;
	TorqueSensorReport torque_sensor_report;
};

class FSRReport{
public:
	double threshold;
	double measuredForce;
};

class LegReport{
public:
	LegReport(int motor_count, int fsr_count);
	FSRReport* fsr_reports;
	JointReport* joint_reports;
	int state;
	Phase phase;
};

class ExoReport{
public:
	LegReport* left_leg;
	LegReport* right_leg;
};


#endif
