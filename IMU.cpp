#include <Arduino.h>
#include "IMU.hpp"
#include "System.hpp"
#include <Adafruit_BNO055_t3.h>
#include <utility/imumaths.h>

IMU::IMU(IMUPins* imu_pins){

  bno = new Adafruit_BNO055(WIRE_BUS, 1, imu_pins->address, I2C_MASTER, imu_pins->imu_slot,
                            I2C_PULLUP_EXT, I2C_RATE_100, I2C_OP_MODE_ISR);

  if (!bno->begin()) {
    Serial.print("No IMU detected");
  }
}

void IMU::calibrate(){
  sensors_event_t event;
  Serial.println("IMU setup... calibrating");
  while (!bno->isFullyCalibrated()) {
    bno->getEvent(&event);

    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);

    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno->getCalibration(&system, &gyro, &accel, &mag);
    /* Optional: Display calibration status */
    Serial.print("\tSys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);

    /* New line for the next sample */
    Serial.println("");

    /* Wait the specified delay before requesting new data */
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}

void IMU::measure(){
  if (imu_measure_limiter.check()){
    euler = bno->getVector(Adafruit_BNO055::VECTOR_EULER);
    imu_measure_limiter.reset();
  }
}

void IMU::getOrientation(double* orientation){
  orientation[0] = euler[0];
  orientation[1] = euler[1];
  orientation[2] = euler[2];
}
