#ifndef SMBATTERY_H
#define SMBATTERY_H
#include <Wire.h>

class SMBattery {
  public:
  void init() {
    Wire.begin();
  }
  int readSOC() {
    int new_soc = readWord(SOC_REG);
    // initialize previous reading
    if (_last_soc == -1 && new_soc >= 0 && new_soc <= 100) {
      _last_soc = new_soc;
    }
    // filter output
    int high = _last_soc + _filter_range;
    int low = _last_soc - _filter_range;
    if (new_soc <= high && new_soc >= low) {
      _last_soc = new_soc;
      return new_soc;
    } else {
      return _last_soc;
    }
  }
  
  int readVoltage() {
    return max(0, min(readWord(VOL_REG), 48));
  }
  private:
  const byte BATT_ADDR = 0xb;
  const byte SOC_REG = 0x0e; 
  const byte VOL_REG = 0x09;
  int _last_soc = -1;
  const int _filter_range = 5;

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
