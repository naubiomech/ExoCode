#include "BleParser.h"
#include "Utilities.h"

BleParser::BleParser()
{
  ;
}

BleMessage BleParser::handle_raw_data(char* buffer, int length)
{
    BleMessage return_msg = BleMessage();
    if (!_waiting_for_data)
    {
        _handle_command(*buffer);
        if (_working_message.is_complete)
        {   
            return_msg.copy(_working_message);
            _working_message.clear();
            _waiting_for_data = false;
        }
    }
    else
    {
        memcpy(&_buffer[_bytes_collected],buffer,length);
        _bytes_collected += length;
        if (_bytes_collected == _working_message.expecting)
        {
            return_msg.copy(_working_message);
            for (int i=0;i<(_working_message.expecting);i+=8)
            {
                double f_tmp = 0;
                memcpy(&f_tmp,&_buffer[i],8);
                return_msg.data[i/8] = (float)f_tmp;
            }
            return_msg.is_complete = true;
            _working_message.clear();
            _waiting_for_data = false;
            _bytes_collected = 0;
        }
        else if (_bytes_collected >= _working_message.expecting)
        {
            _working_message.clear();
            _waiting_for_data = false;
            _bytes_collected = 0;
        }
    }
    return return_msg;
}

void BleParser::package_raw_data(char* data, int start_length)
{
    ;
}

/*
 * Private Functions
 */

void BleParser::_handle_command(char command)
{
    int length = -1;
    //Get the ammount of characters to wait for
    for(int i=0; i < sizeof(ble::commands)/sizeof(ble::commands[0]); i++)
    {
        if(command == ble::commands[i].command)
        {
            length = ble::commands[i].length;
            break;
        }
    }
    if (length == -1)
    {
        // Didnt find command in list
        _working_message.clear();
        Serial.println("BleParser::_handle_command->Command is not in list");
    }
    else
    {
        _waiting_for_data = (length != 0);
        _working_message.command = command;
        // Must multiply by 8 to account for the number of bytes to expect
        _working_message.expecting = length * 8;
        _working_message.is_complete = !_waiting_for_data;
    }
}
