#ifndef BLEMESSAGE_H
#define BLEMESSAGE_H

#include <vector>
static const int _max_size = 10;

class BleMessage
{
    public:
        BleMessage();
        void clear();
        void copy(BleMessage* n);
        
        char command = 0;
        int expecting = 0;
        bool is_complete = false;
        float data[_max_size];
    private:
        int _size = 0;
};

#endif
