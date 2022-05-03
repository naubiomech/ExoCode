#include "BleParser.h"
#include "Utilities.h"
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
          Serial.println("BleParser::handle_raw_data->Got command with no data");
          //Command with no data
          return_msg.copy(_working_message);
        }
        else
        {
          Serial.println("BleParser::handle_raw_data->Waiting for data, new command");
          return_msg.copy(_empty_message);
        }
    }
    else if(_waiting_for_data)
    {
        Serial.println("BleParser::handle_raw_data->Waiting for data");
        //Expecting data
        for (int i = _buffer.size(); i < (_buffer.size()+length); i++)
        {
            _buffer[i] = buffer[i];
        }
        Serial.print("BleParser::handle_raw_data->Got data: ");
        for (int i=0; i < _buffer.size(); i++)
        {
          Serial.print(_buffer[i]);
          Serial.print("\t");
        }
        Serial.println();
        if (_buffer.size() == _working_message.expecting)
        {
          Serial.println("BleParser::handle_raw_data->Got all data");
          //We have all of the data, convert to float
          _working_message.data = _convert_to_floats(_buffer, _buffer.size(), _working_message.expecting/8);
          //Format return message
          _working_message.is_complete = true;
          return_msg.copy(_working_message);
          //Reset logic
          _working_message.clear();
          _waiting_for_data = false;
          _buffer.clear();
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

/*
 * Private Functions
 */

void BleParser::_handle_command(char command)
{
    int length = -1;
    Serial.print("BleParser::_handle_command->Looking for: ");
    Serial.println(command);
    //Get the ammount of characters to wait for
    for(int i=0; i < sizeof(ble::commands)/sizeof(ble::commands[0]); i++)
    {
        Serial.print("BleParser::_handle_command->Checking: ");
        Serial.print(ble::commands[i].command); Serial.print("\t");
        Serial.println(ble::commands[i].length);
        if(command == ble::commands[i].command)
        {
            length = ble::commands[i].length;
            break;
        }
    }
    if (length == -1)
    {
        Serial.println("BleParser::_handle_command->Didn't find command");
        _working_message.clear();
    }
    else
    {
        Serial.print("BleParser::_handle_command->Got valid command: ");
        Serial.println(command);
        _waiting_for_data = (length != 0);
        _working_message.command = command;
        // Must multiply by 8 to account for the number of bytes to expect
        _working_message.expecting = length * 8;
        _working_message.is_complete = (length == 0);
    }
}

    /*
    * Converts char bytes to floats
    */
    std::vector<float> BleParser::_convert_to_floats(std::vector<char> buffer, int buffer_len, int n_floats)
    {   
        std::vector<float> ret;
        ret.reserve(n_floats);
        int bytes_per_float = sizeof(char)*buffer_len/n_floats;
        for (int i=0; i<ret.size(); i++)
        {
            float* dst;
            char* src = &buffer[i*bytes_per_float];
            memcpy(dst, src, bytes_per_float);
            ret[i] = (float)*dst;
        }

        return ret;
    }    
