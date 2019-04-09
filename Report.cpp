#include "Report.hpp"
#include "JointSelect.hpp"
#include <cstddef>

template <class T>
void* deleteReportList(LinkedList<T>* reports){
  ListIterator<T> iter = reports->getIterator();
  while  (iter.hasNext()){
    delete iter.next();
  }
  reports->clear();
  return NULL;
}

Report::~Report(){}

SensorReport::~SensorReport(){
  deleteReportList(&fsr_reports);
  deleteReportList(&imu_reports);
}

LegReport::LegReport(){
  sensor_reports = NULL;
}

LegReport::LegReport(LinkedList<JointReport*>& joint_reports, SensorReport* sensor){
  sensor_reports = sensor;
  this->joint_reports = joint_reports;
}

LegReport::~LegReport(){
  deleteReportList(&joint_reports);
  delete sensor_reports;
}

JointReport* LegReport::getJointReport(JointID id){
  return joint_reports[id];
}

JointReport::JointReport(TorqueSensorReport* torque_sensor, MotorReport* motor, PotReport* pot){
  this->motor_report = motor;
  this->torque_sensor_report = torque_sensor;
  this->pot_report = pot;
}

JointReport::JointReport(){
  motor_report = NULL;
  torque_sensor_report = NULL;
  pot_report = NULL;
}

JointReport::~JointReport(){
  delete motor_report;
  delete torque_sensor_report;
  delete pot_report;
}

ExoReport::ExoReport(){
  left_leg = NULL;
  right_leg = NULL;
}

ExoReport::ExoReport(LegReport* left, LegReport* right){
  left_leg = left;
  right_leg = right;
}

ExoReport::~ExoReport(){
  delete left_leg;
  delete right_leg;
}

LegReport* ExoReport::getAreaReport(AreaID id){

  if (id == joint_select.area_id.LEFT_LEG){
    return left_leg;
  } else if (id == joint_select.area_id.RIGHT_LEG){
    return right_leg;
  }
  return NULL;
}
