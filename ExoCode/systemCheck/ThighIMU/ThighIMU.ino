// Teensy code to connect to two thigh IMUs
#include "src/I2CHandler.h"
#include "src/Utilities.h"

#include "src/ThIMU.h"

ThIMU left = ThIMU(true);

void setup() {
  Serial.begin(115200);
  delay(100);
  while(!Serial);

  if (!left.init()) {
    Serial.println("Failed Left Init");
  }
  
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

  data = left.read_data();

  Serial.print("Got: ");
  Serial.println(data);
  
  
  // Try to read at 100Hz
  delay(100);
}
