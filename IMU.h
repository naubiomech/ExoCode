#ifndef IMU_HEADER
#define IMU_HEADER

#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_t3.h>
#include <utility/imumaths.h>

sensors_event_t event;
imu::Vector<3> eulerTwo;

Adafruit_BNO055 bnoTwo = Adafruit_BNO055(WIRE1_BUS, 1, 0x28, I2C_MASTER, I2C_PINS_3_4, I2C_PULLUP_EXT, I2C_RATE_100, I2C_OP_MODE_ISR);

bool IMU_flag;
#endif
