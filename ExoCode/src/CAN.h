/* 
 *  Using the FlexCAN library included by Teensyduino.
 *  This class uses the singleton design pattern. Read about it here: https://www.tutorialspoint.com/Explain-Cplusplus-Singleton-design-pattern
 */

#ifndef CAN_H
#define CAN_H

#include "Arduino.h"
// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
// #include "FlexCAN.h"

// class CAN 
// {
//     public:
//         static CAN* getInstance()
//         {
//             static CAN* instance = new CAN;
//             return instance;
//         }

//         void send(CAN_message_t msg)
//         {
//             msg.ext = 0;
//             msg.len = 8;
//             if(!Can0.write(msg)) 
//             {
//                 Serial.print("Error Sending");
//                 Serial.print(msg.id);
//                 Serial.print("\n");
//             }
//         }

//         CAN_message_t read()
//         {
//             CAN_message_t msg;
//             Can0.read(msg);
//             return msg;
//         }

//     private:
//         static CAN* instance;

//         CAN()
//         {   
//             Can0.setTxBufferSize(16);
//             Can0.begin(1000000);
//         }
// };

#include "FlexCAN_T4.h"
#if defined(ARDUINO_TEENSY36)
static FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
#elif defined(ARDUINO_TEENSY41)
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
#endif

class CAN 
{
    public:
        static CAN* getInstance()
        {
            static CAN* instance = new CAN;
            return instance;
        }

        void send(CAN_message_t msg)
        {
            if(!Can0.write(msg)) 
            {
                // Serial.print("Error Sending");
                // Serial.print(msg.id);
                // Serial.print("\n");
            }
        }

        CAN_message_t read()
        {
            CAN_message_t msg;
            Can0.read(msg);
            return msg;
        }

    private:
        CAN()
        {   
            Can0.begin();
            Can0.setBaudRate(1000000);
        }
};

#endif
#endif