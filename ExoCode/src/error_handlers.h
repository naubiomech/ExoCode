#ifndef ERROR_HANDLERS_H
#define ERROR_HANDLERS_H

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "ExoData.h"
#include "Exo.h"
#include "error_types.h"

// Handles errors
namespace error_handlers
{
    // Helper to report the errors
    void report(uint16_t error_code)
    {
        Serial.print("Error code: ");
        Serial.println(error_code);
    }
    void soft(Exo* exo, ExoData* exo_data, uint16_t error_code)
    {
        // TODO: Recalibrate/Warn user

        report(error_code);
        return;
    }
    void hard(Exo* exo, ExoData* exo_data, uint16_t error_code)
    {
        // TODO: Change all controllers to zero torque/statis

        report(error_code);
        return;
    }
    void fatal(Exo* exo, ExoData* exo_data, uint16_t error_code)
    {
        // TODO: Permanently disable motor(s)
        // Ensure that the motors are not reenabled
        exo_data->for_each_joint([](JointData* joint_data) {joint_data->motor.enabled = 0;});
        // Disable the motors
        exo->left_leg.disable_motors();
        exo->right_leg.disable_motors();
        
        report(error_code);
        return;
    }
}

#endif

#endif // defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)