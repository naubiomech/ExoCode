
int readBatteryVoltage() { // Read the battery voltage from the INA219 over I2C
  WireObj.beginTransmission(INA219_ADR); // Start a new transmission
  WireObj.write(INA219_BUS);             // Specify the register we want to read (bus/battery voltage)
  WireObj.endTransmission();             // End transmission
  WireObj.requestFrom(INA219_ADR,2);     // Request two bytes of data from the INA219
  int BatVolt = WireObj.read();          // Get the first byte of the battery voltage
  BatVolt = BatVolt << 8;                // Shift the data over by 8 bits since the voltage is 16 bits total
  int BatVolt2 = WireObj.read();         // Get the second byte
  BatVolt = BatVolt|BatVolt2;            // Not the second 8 bits with the previous shifted value to finish
  BatVolt = BatVolt >> 3;                // Drop the last three bits of the voltage as required by the documentation
  BatVolt = BatVolt*BusLSB;              // Multiply the voltage by the calibration value to finalize the battery voltage calculation
  return BatVolt;
}

int readShuntVoltage() { // Read the shunt resistor voltage from the INA219 over I2C
  WireObj.beginTransmission(INA219_ADR);   // Start a new transmission
  WireObj.write(INA219_SHUNT);             // Specify the register we want to read (shunt voltage)
  WireObj.endTransmission();               // End transmission
  WireObj.requestFrom(INA219_ADR,2);       // Request two bytes of data from the INA219
  int ShuntVolt = WireObj.read();          // Get the first byte of the shunt voltage
  ShuntVolt = ShuntVolt << 8;              // Shift the data over by 8 bits since the voltage is 16 bits total
  int ShuntVolt2 = WireObj.read();         // Get the second byte
  ShuntVolt = ShuntVolt|ShuntVolt2;        // Not the second 8 bits with the previous shifted value to finish
  ShuntVolt = ShuntVolt*ShuntLSB;          // Multiply the voltage by the calibration value to finalize the battery voltage calculation
  return ShuntVolt;
}

int calcShuntCurrent() { // Calculate the current from the shunt resistor voltage
  int ShuntVolt = readShuntVoltage();    // Read the shunt voltage
  int ShuntCur = ShuntVolt/0.002;        // Divide the shunt voltage by the shunt resistance (2mOhm)
  return ShuntCur;
}
