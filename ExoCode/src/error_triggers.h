#ifndef ERROR_TRIGGERS_H
#define ERROR_TRIGGERS_H

#include "ExoData.h"
#include "Exo.h"
#include "error_types.h"

// Returns true if error
namespace error_triggers
{
    uint8_t soft(Exo* exo, ExoData* exo_data)
    {
        if ((millis() > 10000) && (millis() < 12000))
        {
            return true;
        }
        return false;
    }
    uint8_t hard(Exo* exo, ExoData* exo_data)
    {
        if ((millis() > 20000) && (millis() < 22000))
        {
            return true;
        }
        return 0;
    }
     uint8_t fatal(Exo* exo, ExoData* exo_data)
    {
        if ((millis() > 30000) && (millis() < 32000))
        {
            return true;
        }
        return 0;
    }
}

#endif