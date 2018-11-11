#include "Report.hpp"

LegReport::LegReport(int joint_count, int fsr_count){
	joint_reports = new JointReport[joint_count];
	fsr_reports = new FSRReport[fsr_count];
}
