#include "Report.hpp"

LegReport::LegReport(int joint_count, int fsr_count, int imu_count){
  joint_report_count = joint_count;
  fsr_report_count = fsr_count;
  imu_report_count = imu_count;

  joint_reports = new JointReport*[joint_count];
  fsr_reports = new FSRReport*[fsr_count];
  imu_reports = new IMUReport*[imu_count];

  for (int i = 0; i < joint_count; i++){
    joint_reports[i] = new JointReport();
  }

  for (int i = 0; i < fsr_count; i++){
    fsr_reports[i] = new FSRReport();
  }

  for (int i = 0; i < imu_count; i++){
    imu_reports[i] = new IMUReport();
  }
}

LegReport::~LegReport(){

  for (int i = 0; i < joint_report_count; i++){
    delete joint_reports[i];
  }

  for (int i = 0; i < fsr_report_count; i++){
    delete fsr_reports[i];
  }

  for (int i = 0; i < imu_report_count; i++){
    delete imu_reports[i];
  }

  delete[] joint_reports;
  delete[] fsr_reports;
  delete[] imu_reports;
}

JointReport::JointReport(){
  motor_report = new MotorReport();
  torque_sensor_report = new TorqueSensorReport();
}

JointReport::~JointReport(){
  delete motor_report;
  delete torque_sensor_report;
}
