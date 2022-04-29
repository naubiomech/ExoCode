#include "BleParser.h"
#include "Arduino.h"

BleParser::BleParser()
{
  ;
}

BleMessage BleParser::handle_raw_data(char* buffer, int length)
{
  Serial.println("BleParser::handle_raw_data->Start");
  BleMessage return_msg = BleMessage();
    if(!_waiting_for_data && (length == 1))
    {
        Serial.println("BleParser::handle_raw_data->!Waiting for data");
        //Expecting a single character representing a command
        _handle_command(buffer[0]);
        if(_working_message.is_complete)
        {
          //Command with no data
          return_msg.copy(_working_message);
        }
        else
        {
          return_msg.copy(_empty_message);
        }
        
    }
    else if(_waiting_for_data)
    {
        Serial.println("BleParser::handle_raw_data->Waiting for data");
        //Expecting data
        for (int i = 0; i <= length; i++)
        {
            _working_message.data[i] = buffer[i];
        }
        if (_working_message.data.size() == _working_message.expecting)
        {
          Serial.println("BleParser::handle_raw_data->Got all data");
          //We have all of the data 
          _working_message.is_complete = true;
          return_msg.copy(_working_message);
          _working_message.clear();
          _waiting_for_data = false;
        }
        else
        {
          return_msg = _empty_message;
        }
    }
    Serial.println("BleParser::handle_raw_data->End");
    return return_msg;
}

void BleParser::package_raw_data(char* data, int start_length)
{
    ;
}

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
        _working_message.clear();
    } 
    else
    {
        _waiting_for_data = (length != 0);
        _working_message.command = command;
        _working_message.expecting = length;
        _working_message.is_complete = (length != 0);
    }
}
