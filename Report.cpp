#include "Report.hpp"

LegReport::LegReport(int motor_count, int fsr_count){
	motor_reports = new MotorReport[motor_count];
	fsr_reports = new FSRReport[fsr_count];
}
