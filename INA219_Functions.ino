
int readBatteryVoltage() { // Read and calculate the battery voltage from the INA219 over I2C
  Wire1.beginTransmission(INA219_ADR); // Start a new transmission
  Wire1.write(INA219_BUS);             // Specify the register we want to read (bus/battery voltage)
  Wire1.endTransmission();             // End transmission
  Wire1.requestFrom(INA219_ADR,2);     // Request two bytes of data from the INA219
  int BatVolt = Wire1.read();          // Get the first byte of the battery voltage
  BatVolt = BatVolt << 8;              // Shift the data over by 8 bytes since the voltage is 16 bytes total
  int BatVolt2 = Wire1.read();
  BatVolt = BatVolt|BatVolt2;             // Not the second 8 bytes with the previous shifted value to finish
  BatVolt = BatVolt >> 3;              // Drop the last three bytes of the voltage as required by the documentation
  BatVolt = BatVolt*BusLSB;            // Multiply the voltage by the calibration value to finalize the battery voltage calculation
  return BatVolt;
}

int readShuntVoltage() { // Read and calculate the shunt resistor voltage from the INA219 over I2C
  Wire1.beginTransmission(INA219_ADR);   // Start a new transmission
  Wire1.write(INA219_SHUNT);             // Specify the register we want to read (shunt voltage)
  Wire1.endTransmission();               // End transmission
  Wire1.requestFrom(INA219_ADR,2);       // Request two bytes of data from the INA219
  int ShuntVolt = Wire1.read();          // Get the first byte of the shunt voltage
  ShuntVolt = ShuntVolt << 8;            // Shift the data over by 8 bytes since the voltage is 16 bytes total
  ShuntVolt |= Wire1.read();             // Not the second 8 bytes with the previous shifted value to finish
  ShuntVolt = ShuntVolt*ShuntLSB;          // Multiply the voltage by the calibration value to finalize the battery voltage calculation
  return ShuntVolt;
}

int calcShuntCurrent() {
  int ShuntVolt = readShuntVoltage();     // Read the shunt voltage
  int ShuntCur = ShuntVolt/0.002;        // Divide the shunt voltage by the shunt resistance (2mOhm)
  return ShuntCur;
}
