#include "Report.hpp"

LegReport::LegReport(int joint_count, int fsr_count, int imu_count){
	joint_reports = new JointReport[joint_count];
	fsr_reports = new FSRReport[fsr_count];
	imu_reports = new IMUReport[imu_count];
	joint_report_count = joint_count;
	fsr_report_count = fsr_count;
	imu_report_count = imu_count;
}

LegReport::~LegReport(){
	delete[] joint_reports;
	delete[] fsr_reports;
	delete[] imu_reports;
}
