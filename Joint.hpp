#ifndef JOINT_HEADER
#define JOINT_HEADER

#include "Report.hpp"

class Joint{
private:
  Motor* motor;
  TorqueSensor* sensor;
public:
  JointReport generateReport();
  void fillReport(JointReport* report);

};

#endif
