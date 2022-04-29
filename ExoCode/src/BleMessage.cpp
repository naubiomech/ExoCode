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

void BleMessage::copy(BleMessage n) 
{ 
    command = n.command; 
    is_complete = n.is_complete;
    data.assign(n.data.begin(), n.data.end());
    expecting = n.expecting;
}
