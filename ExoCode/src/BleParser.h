/* Class to parse BLE messages. This class extracts data from the BLE messages. 

   Constructor: Default

   Chance Cuddeback 2022
*/

#ifndef BLEPARSER_H
#define BLEPARSER_H

#include "BleMessage.h"
#include "ble_commands.h"
#include <vector>


class BleParser
{
    public:
        BleParser();
        /* Unpacks the data from the gui. Returns true if the message is complete */
        BleMessage handle_raw_data(char* buffer, int length);
        
        /* Packs data to be sent to the GUI, modifies the char array that you send it. returns final length */
        void package_raw_data(char* data, int start_length);
    private:
        const BleMessage _empty_message = BleMessage();
        BleMessage _working_message = BleMessage();
        bool _waiting_for_data = false;
        std::vector<char> _buffer;

        void _handle_command(char command);
};


#endif
