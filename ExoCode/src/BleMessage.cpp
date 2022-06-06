#if defined(ARDUINO_ARDUINO_NANO33BLE)
#include "BleMessage.h"

BleMessage::BleMessage() 
{ 
    data.reserve(_max_size);
}

void BleMessage::clear() 
{
    command = 0;
    is_complete = false;
    data.clear();
    expecting = 0;
}

void BleMessage::copy(BleMessage* n) 
{ 
    command = n->command; 
    is_complete = n->is_complete;
    data.clear();
    for (int i=0; i<n->data.size();i++)
    {
        data.push_back(n->data[i]);
    }
    expecting = n->expecting;
}
#endif