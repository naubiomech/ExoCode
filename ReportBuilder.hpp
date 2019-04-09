#include "Report.hpp"

class LegReportBuilder;
class ExoReportBuilder{
private:
  LegReport* left_leg_report;
  LegReport* right_leg_report;
public:
  ExoReportBuilder();
  void setLeftLegReport(LegReport* left_leg_report);
  void setRightLegReport(LegReport* right_leg_report);
  ExoReport* build();
};

class LegReportBuilder{
private:
  SensorReport* sensor_report;
  LinkedList<JointReport*> joint_reports;
public:
  LegReportBuilder();
  void addFSRReport();
  void addIMUReport();
  void addJointReport(bool include_pot);
  LegReport* build();
};
