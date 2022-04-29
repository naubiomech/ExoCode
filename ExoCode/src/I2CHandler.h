/* 

   This class is using the singleton pattern. To use you must first get a referance to the singleton instance.
   Do this with the line 'I2C* instance = I2C::get_instance();'. Then you can read with 'instance->read_i2c(...)'

   Chance Cuddeback 2022
*/

#ifndef I2CHANDLER_H
#define I2CHANDLER_H

#include "Wire.h"

class I2C
{
    public:
        static I2C* get_instance()
        {
            static I2C* instance = new I2C;
            return instance;
        }

        void read_i2c(uint8_t* ret, uint8_t addr, uint8_t reg, uint8_t len)
        {
            Wire.beginTransmission(addr);
            Wire.write(reg);
            Wire.endTransmission();
            Wire.requestFrom(addr, 2, false);
            for (uint8_t i=0; i<len; i++)
            {
                ret[i] = Wire.read();
            }
            Wire.endTransmission();
        }

    private:
        I2C()
        {
          Wire.begin();
        }
};


namespace i2c_cmds
{
    namespace get_battery_voltage
    {
        const uint8_t addr = 0x40;
        const uint8_t reg = 0x02;
        const uint8_t len = 2;
    }
    namespace get_battery_soc
    {
        const uint8_t addr = 0xb;
        const uint8_t reg = 0x0e;
        const uint8_t len = 2;
    }
}

#endif
