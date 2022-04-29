#ifndef BLEMESSAGE_H
#define BLEMESSAGE_H

#include <vector>


class BleMessage
{
    public:
        BleMessage();
        void clear();
        void copy(BleMessage n);
        
        char command = 0;
        int expecting = 0;
        bool is_complete = false;
        std::vector<float> data;
    private:
        static const int _max_size = 10;
};

#endif
