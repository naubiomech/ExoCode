#ifndef BLE_COMMANDS_H
#define BLE_COMMADNS_H

typedef struct
{
    char command;
    int length;  
} ble_command_t;

namespace names
{
    static char start    = 'E';
    static char stop     = 'G';
    static char cal_trq  = 'H';
    static char cal_fsr  = 'L';
    static char new_trq  = 'F';
    static char assist   = 'c';
    static char ressist  = 'S';
};

namespace ble
{
    static ble_command_t commands[] = 
    {
        {names::start,    0},
        {names::stop,     0},
        {names::cal_trq,  0},
        {names::cal_fsr,  0},
        {names::assist,   0},
        {names::ressist,  0},
        {names::new_trq,  4},

    };
};

#endif