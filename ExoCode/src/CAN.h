/* 
 *  Using the FlexCAN library included by Teensyduino.
 *  This class uses the singleton design pattern. Read about it here: https://www.tutorialspoint.com/Explain-Cplusplus-Singleton-design-pattern
 */

#ifndef CAN_H
#define CAN_H

#include "Arduino.h"
#include "FlexCAN.h"

class CAN 
{
    public:
        static CAN* getInstance()
        {
            if (!instance)
            {
                instance = new CAN;
            }
            return instance;
        }

        void send(uint32_t id, uint8_t data[8])
        {
            CAN_message_t msg;
            msg.ext = 0;
            msg.len = 8;
            msg.id = id;
            msg.buf[0] = data[0];
            msg.buf[1] = data[1];
            msg.buf[2] = data[2];
            msg.buf[3] = data[3];
            msg.buf[4] = data[4];
            msg.buf[5] = data[5];
            msg.buf[6] = data[6];
            msg.buf[7] = data[7];
            Can0.write(msg);
        }

        uint32_t read(uint8_t data[8])
        {
            CAN_message_t msg;
            Can0.read(msg);
            for(int i=0; i<8; i++)
            {
                data[i] = msg.buf[i];
            }
            return msg.id;
        }

    private:
        static CAN* instance;

        CAN()
        {
            Can0.begin();
        }
};

CAN* CAN::instance = 0;

#endif