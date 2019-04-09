#include "ReportBuilder.hpp"

ExoReportBuilder::ExoReportBuilder(){
  left_leg_report = NULL;
  right_leg_report = NULL;
}

void ExoReportBuilder::setLeftLegReport(LegReport* left_leg_report){
  this->left_leg_report = left_leg_report;
}

void ExoReportBuilder::setRightLegReport(LegReport* right_leg_report){
  this->right_leg_report = right_leg_report;
}

ExoReport* ExoReportBuilder::build(){
  return new ExoReport(left_leg_report, right_leg_report);
}

LegReportBuilder::LegReportBuilder(){
  sensor_report = new SensorReport();
}

void LegReportBuilder::addFSRReport(){
  sensor_report->fsr_reports.append(new FSRReport());
}

void LegReportBuilder::addIMUReport(){
  sensor_report->imu_reports.append(new IMUReport());
}

void LegReportBuilder::addJointReport(bool include_pot){
  TorqueSensorReport* torque_sensor = new TorqueSensorReport();
  MotorReport* motor = new MotorReport();
  PotReport* pot = include_pot ? new PotReport() : NULL;
  joint_reports.append(new JointReport(torque_sensor, motor, pot));
}

LegReport* LegReportBuilder::build(){
  return new LegReport(joint_reports, sensor_report);
}
