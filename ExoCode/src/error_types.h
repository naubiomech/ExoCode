#ifndef ERROR_TYPES_H
#define ERROR_TYPES_H

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "ExoData.h"
#include "Exo.h"

typedef void (*error_handler_t) (Exo*, ExoData*, uint16_t error_code);

typedef uint8_t (*error_trigger_t) (Exo*, ExoData*);

enum ErrorCodes
{
    NO_ERROR = 0,
    SOFT_ERROR, // General Soft Error
    HARD_ERROR, // General Hard Error
    FATAL_ERROR, // General Fatal Error
    // Soft Errors
    POOR_CALIBRATION,
    // Hard Errors
    POOR_STATE_VARIANCE,
    // Fatal Errors
    POOR_TRANSMISSION_EFFICIENCY,
    MOTOR_POSTION_OUT_OF_BOUNDS,
    JOINT_POSITION_OUT_OF_BOUNDS,
    TORQUE_OUT_OF_BOUNDS,
    TRACKING_ERROR,
    ERROR_CODE_LENGTH
};

#endif

#endif // defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)