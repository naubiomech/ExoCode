#ifndef BLE_COMMANDS_H
#define BLE_COMMADNS_H

#include "ExoData.h"

typedef struct
{
    char command;
    int length;  
} ble_command_t;

namespace names
{
    // Recieved Commands
    static const char start       = 'E';
    static const char stop        = 'G';
    static const char cal_trq     = 'H';
    static const char cal_fsr     = 'L';
    static const char new_trq     = 'F';
    static const char new_fsr     = 'R';
    static const char assist      = 'c';
    static const char resist      = 'S';
    static const char motors_on   = 'x';
    static const char motors_off  = 'w';
    static const char mark        = 'N';

    // Sending Commands
    static const char send_real_time_data = '?';
    static const char send_batt           = '~';
    static const char send_cal_done       = 'n';
    static const char send_error_count    = 'w';
    static const char send_trq_cal        = 'H';
    static const char send_step_count     = 's';
};

namespace ble
{
    static const ble_command_t commands[] = 
    {
        // Recieved Commands
        {names::start,      0},
        {names::stop,       0},
        {names::cal_trq,    0},
        {names::cal_fsr,    0},
        {names::assist,     0},
        {names::resist,     0},
        {names::motors_on,  0},
        {names::motors_off, 0},
        {names::mark,       0},
        {names::new_fsr,    2},
        {names::new_trq,    4},
        
        // Sending Commands
        {names::send_batt, 1},
        {names::send_real_time_data, 9},
        {names::send_error_count, 1},
        {names::send_cal_done, 0},
        {names::send_trq_cal, 2},
        {names::send_step_count, 2}
    };
};

// All command handlers should have static linkage, return void, and accept a pointer to ExoData, ie "inline static void my_handler(ExoData* data)"
namespace handlers
{
    inline static void start(ExoData* data)
    {
        
    }
    inline static void stop(ExoData* data)
    {

    }
    inline static void cal_trq(ExoData* data)
    {

    }
    inline static void cal_fsr(ExoData* data)
    {

    }
    inline static void assist(ExoData* data)
    {

    }
    inline static void resist(ExoData* data)
    {

    }
    inline static void motors_on(ExoData* data)
    {

    }
    inline static void motors_off(ExoData* data)
    {

    }
    inline static void mark(ExoData* data)
    {

    }
    inline static void new_trq(ExoData* data)
    {

    }
    inline static void new_fsr(ExoData* data)
    {

    }

}

#endif