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
        // TODO: Recalibrate/Warn user
        //Serial.println("Soft error");
        return;
    }
    void hard(Exo* exo, ExoData* exo_data)
    {
        // TODO: Change all controllers to zero torque/statis
        //Serial.println("Hard error");
        return;
    }
    void fatal(Exo* exo, ExoData* exo_data)
    {
        // TODO: Test
        exo_data->for_each_joint([](JointData* joint_data) {
            joint_data->motor.enabled = 0;
        });
        Serial.println("Fatal error");
        return;
    }
}

#endif

#endif // defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)