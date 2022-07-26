/* Class to parse BLE messages. This class extracts data from the BLE messages. 

   Constructor: Default

   Chance Cuddeback 2022
*/

#ifndef BLEPARSER_H
#define BLEPARSER_H

#include "Arduino.h"
#include "BleMessage.h"
#include "ble_commands.h"
#include "GattDb.h"
#include <vector>


class BleParser
{
    public:
        BleParser();
        /* Unpacks the data from the gui. Returns true if the message is complete */
        BleMessage* handle_raw_data(char* buffer, int length);
        
        /* Packs data to be sent to the GUI, modifies the char array that you send it. returns final length */
        int package_raw_data(byte* buffer, BleMessage &msg);
        
    private:
        const BleMessage _empty_message = BleMessage();
        BleMessage _working_message = BleMessage();
        bool _waiting_for_data = false;
        int _bytes_collected = 0;
        byte _buffer[64];

        void _handle_command(char command);
};


#endif