#ifndef SMBATTERY_H
#define SMBATTERY_H
#include <Wire.h>

class SMBattery {
  public:
  void init() {
    Wire.begin();
  }
  int readSOC() {
    return readWord(SOC_REG);
  }
  int readVoltage() {
    return readWord(VOL_REG);
  }
  private:
  const byte BATT_ADDR = 0xb;
  const byte SOC_REG = 0x0e; 
  const byte VOL_REG = 0x09;

  int readWord(byte command) {
    Wire.beginTransmission(BATT_ADDR);
    Wire.write(command);
    Wire.endTransmission();
    Wire.requestFrom(BATT_ADDR,2,false);
    byte a = Wire.read();
    byte b = Wire.read();
    Wire.endTransmission();
    int ret = (((int)b << 8) | (int)a);
    return ret; 
  }
};
#endif
