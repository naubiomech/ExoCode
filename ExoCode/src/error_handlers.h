#ifndef ERROR_HANDLERS_H
#define ERROR_HANDLERS_H

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "ExoData.h"
#include "Exo.h"
#include "error_types.h"

// Handles errors
namespace error_handlers
{
    void soft(Exo* exo, ExoData* exo_data)
    {
        //Serial.println("Soft error");
        return;
    }
    void hard(Exo* exo, ExoData* exo_data)
    {
        //Serial.println("Hard error");
        return;
    }
     void fatal(Exo* exo, ExoData* exo_data)
    {
        //Serial.println("Fatal error");
        return;
    }
}

#endif

#endif // defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)