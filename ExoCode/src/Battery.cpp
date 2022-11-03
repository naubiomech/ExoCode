#include "Battery.h"
#include "Arduino.h"
#if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)

#define BATTERY_DEBUG 0

void SmartBattery::init() {;}
float SmartBattery::get_parameter()
{
    I2C* instance = I2C::get_instance();
    instance->read_i2c(data, i2c_cmds::smart::get_battery_voltage::addr, i2c_cmds::smart::get_battery_voltage::reg, i2c_cmds::smart::get_battery_voltage::len);
    uint8_t voltage = (data[0] << 8) | data[1];
    voltage = voltage >> 3;
    return float(voltage);
}

void RCBattery::init() 
{
    I2C* instance = I2C::get_instance();
    instance->write_i2c(i2c_cmds::rc::calibrate::addr, i2c_cmds::rc::calibrate::reg, i2c_cmds::rc::calibrate::val);
}
float RCBattery::get_parameter()
{
    // Battery Voltage, could get shunt voltage and calculate current for funsies
    I2C* instance = I2C::get_instance();
    instance->read_i2c(data, i2c_cmds::rc::get_battery_voltage::addr, i2c_cmds::rc::get_battery_voltage::reg, i2c_cmds::rc::get_battery_voltage::len);
    
    #if BATTERY_DEBUG
    Serial.print("RCBattery::get_parameter->Raw Data: ");
    Serial.print(data[0]);
    Serial.print(" ");
    Serial.println(data[1]);
    #endif

    uint8_t voltage = (data[0] << 8) | data[1];
    voltage = voltage >> 3;
    voltage = voltage * BusLSB * 10000;

    #if BATTERY_DEBUG
    Serial.print("RCBattery::get_parameter->Battery Voltage: ");
    Serial.println(voltage);
    #endif

    return float(voltage);
}
#endif