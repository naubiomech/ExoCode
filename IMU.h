#ifndef IMU_HEADER
#define IMU_HEADER

#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_t3.h>
#include <utility/imumaths.h>

void setupIMU(Adafruit_BNO055* bno);
void calibrateIMU(Adafruit_BNO055* bno);
void updateIMU(Adafruit_BNO055* bno);
#endif
