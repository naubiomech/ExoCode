/*
 * 
 * P. Stegall Jan. 2022
*/

#include "ExoData.h"


/*
 * Constructor for the exo data.
 * Takes the array from the INI parser.
 * Stores the exo status, and the sync LED state.
 * TODO: decide if we want to change the sync LED state to a pointer.  Or bring the whole sync LED object into the exo.
 * Uses an initializer list for the leg data. 
 */
ExoData::ExoData(uint8_t* config_to_send) 
: left_leg(true, config_to_send)  // using initializer list for member objects.
, right_leg(false, config_to_send)
{
    this->status = ExoStatus::not_enabled;
    this->sync_led_state = false;
    
};



