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

        void send(CAN_message_t msg)
        {
            msg.ext = 0;
            msg.len = 8;
            Can0.write(msg);
        }

        CAN_message_t read()
        {
            CAN_message_t msg;
            Can0.read(msg);
            return msg;
        }

    private:
        static CAN* instance;

        CAN()
        {
            Can0.begin(1000000);
        }
};

CAN* CAN::instance = 0;

#endif