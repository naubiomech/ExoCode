#include "Battery.h"
#include "Arduino.h"

void SmartBattery::init() {;}
float SmartBattery::get_parameter()
{
    I2C* instance = I2C::get_instance();
    instance->read_i2c(data, i2c_cmds::get_battery_voltage::addr, i2c_cmds::get_battery_voltage::reg, i2c_cmds::get_battery_voltage::len);
    uint8_t voltage = (data[0] << 8) | data[1];
    voltage = voltage >> 3;
    return float(voltage);
}

void RCBattery::init() {;}
float RCBattery::get_parameter()
{
    //TODO: Populate with INA219 reading
    static float battery_percent = 101*10;
    if (battery_percent < 1*10)
    {
        battery_percent = 101*10;
    }

    battery_percent = battery_percent - 10;
    return battery_percent;
}
