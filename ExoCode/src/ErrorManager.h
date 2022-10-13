#ifndef ERRORMANAGER_H
#define ERRORMANAGER_H

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "error_types.h"

/**
 * @brief Class manages the calling of error handlers and triggers. Only the control MCU is required to use this.
 * The triggers and handlers must be assigned before check is ran. They are defined in error_triggers.h and error_handlers.h.
 * Check should be called every loop, and should not, severely, increase the loop time. 
 */
class ErrorManager
{
    public:
        ErrorManager(Exo* exo, ExoData* exo_data);
        void check();

        void assign_handlers(error_handler_t soft, error_handler_t hard, error_handler_t fatal);
        void assign_triggers(error_trigger_t soft, error_trigger_t hard, error_trigger_t fatal);

    private:
        Exo* _exo;
        ExoData* _data;

        bool _set_handlers;
        bool _set_triggers;

        error_handler_t _soft_handler = NULL;
        error_handler_t _hard_handler = NULL;
        error_handler_t _fatal_handler = NULL;

        error_trigger_t _soft_trigger = NULL;
        error_trigger_t _hard_trigger = NULL;
        error_trigger_t _fatal_trigger = NULL;
};

#endif

#endif // defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)