#ifndef EXO_REPORT_HEADER
#define EXO_REPORT_HEADER
#include "Phase.hpp"

class MotorReport{
public:
	double measuredTorque;
	double pid_setpoint;
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
	MotorReport* motor_reports;
	int state;
	Phase phase;
};

class ExoReport{
public:
	LegReport* left_leg;
	LegReport* right_leg;
};


#endif
