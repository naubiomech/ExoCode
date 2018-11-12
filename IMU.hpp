#ifndef IMU_HEADER
#define IMU_HEADER
#include "Pins.hpp"
#include <Metro.h>
#include <Adafruit_BNO055_t3.h>
#include "Parameters.hpp"
#include "Report.hpp"

class IMU{
private:
  Adafruit_BNO055* bno;
  double bearings[3] = {0};
  Metro imu_measure_limiter = Metro(BNO055_SAMPLERATE_DELAY_MS);

public:
  IMU(IMUPins* imu_pins);
  void calibrate();
  void measure();
  void getOrientation(double* orientation);
  IMUReport* generateReport();
  void fillReport(IMUReport* report);
  void fillLocalReport(IMUReport* report);
};

#endif
