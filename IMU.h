#ifndef IMU_HEADER
#define IMU_HEADER

#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_t3.h>
#include <utility/imumaths.h>

const int BNO055_SAMPLERATE_DELAY_MS = 100;
sensors_event_t event;
imu::Vector<3> euler;

double stability_trq;
const double stability_trq_gain = 0.2;

Adafruit_BNO055 bno = Adafruit_BNO055(WIRE_BUS, 1, BNO055_ADDRESS_A, I2C_MASTER, IMU_1_PINS, I2C_PULLUP_EXT, I2C_RATE_100, I2C_OP_MODE_ISR);

bool IMU_flag;

void calibrateIMU(Adafruit_BNO055 bno);

#endif
