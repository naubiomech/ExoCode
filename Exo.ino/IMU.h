
#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_t3.h>
#include <utility/imumaths.h>

sensors_event_t event;
imu::Vector<3> eulerTwo;

bool IMU_flag;
