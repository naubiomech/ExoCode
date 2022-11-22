// Teensy code to connect to two thigh IMUs
#include "src/I2CHandler.h"
#include "src/Utilities.h"

#include <Wire.h>


void setup() {
  Serial.begin(115200);
  delay(100);
  //while(!Serial);

  // Start Master I2C
  Wire1.begin();

  // Handshake 
  Wire1.beginTransmission(i2c_cmds::thigh_imu::left_addr);
  Wire1.write(i2c_cmds::thigh_imu::handshake::reg);
  Wire1.endTransmission();
  Wire1.requestFrom(i2c_cmds::thigh_imu::left_addr, i2c_cmds::thigh_imu::get_angle::len, false);
  uint8_t val = Wire1.read();
  if (val == 0x01) {
    Serial.println("Successful handshake!");
  } else {
    Serial.println("Handshake failed!");
  }
  Wire1.endTransmission();
  
}

void loop() {
  //static I2C* i2c = I2C::get_instance();
  static uint8_t data;

//  Serial.print("Reading from I2C device: ");
//  Serial.print(i2c_cmds::thigh_imu::left_addr);
//  Serial.print(" at register: ");
//  Serial.print(i2c_cmds::thigh_imu::get_angle::reg);
//  Serial.print(" with length: ");
//  Serial.println(i2c_cmds::thigh_imu::get_angle::len);

  Wire1.beginTransmission(i2c_cmds::thigh_imu::left_addr);
  Wire1.write(i2c_cmds::thigh_imu::get_angle::reg);
  Wire1.endTransmission();
  Wire1.requestFrom(i2c_cmds::thigh_imu::left_addr, i2c_cmds::thigh_imu::get_angle::len, false);
  data = Wire1.read();
  Wire1.endTransmission();

  Serial.print("Got: ");
  Serial.println(data);
  
  
  // Try to read at 100Hz
  delay(100);
}
