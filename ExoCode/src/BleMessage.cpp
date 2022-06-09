#if defined(ARDUINO_ARDUINO_NANO33BLE)
#include "BleMessage.h"

BleMessage::BleMessage() 
{ 
    
}

void BleMessage::clear() 
{
    command = 0;
    is_complete = false;
    _size = 0;
    expecting = 0;
}

void BleMessage::copy(BleMessage* n) 
{ 
    command = n->command; 
    is_complete = n->is_complete;
    _size = n->_size;
    expecting = n->expecting;
    for (int i=0; i<expecting;i++)
    {
        data[i] = (n->data[i]);
    }  
}
#endif