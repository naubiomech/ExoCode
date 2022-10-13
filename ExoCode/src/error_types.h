#ifndef ERROR_TYPES_H
#define ERROR_TYPES_H

#include "ExoData.h"
#include "Exo.h"

typedef void (*error_handler_t) (Exo*, ExoData*);

typedef uint8_t (*error_trigger_t) (Exo*, ExoData*);

typedef enum error_levels_t
{
    NO_ERROR = 0,
    SOFT_ERROR = 1,
    HARD_ERROR = 2,
    FATAL_ERROR = 3
} error_levels_t;

#endif