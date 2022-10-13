#include "ErrorManager.h"



ErrorManager::ErrorManager(Exo* exo, ExoData* exo_data)
: _exo(exo), _data(exo_data)
{
    _set_handlers = false;
    _set_triggers = false;
}

void ErrorManager::check()
{
    if (!_set_handlers || !_set_triggers)
    {
        return;
    }

    if (_soft_trigger(_exo, _data))
    {
        _soft_handler(_exo, _data);
    }
    else if (_hard_trigger(_exo, _data))
    {
        _hard_handler(_exo, _data);
    }
    else if (_fatal_trigger(_exo, _data))
    {
        _fatal_handler(_exo, _data);
    }
}

void ErrorManager::assign_handlers(error_handler_t soft, error_handler_t hard, error_handler_t fatal)
{
    //TODO: Check if null functions
    _soft_handler = soft;
    _hard_handler = hard;
    _fatal_handler = fatal;
    _set_handlers = true;
}

void ErrorManager::assign_triggers(error_trigger_t soft, error_trigger_t hard, error_trigger_t fatal)
{
    //TODO: Check if null functions
    _soft_trigger = soft;
    _hard_trigger = hard;
    _fatal_trigger = fatal;
    _set_triggers = true;
}
