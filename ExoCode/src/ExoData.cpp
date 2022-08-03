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
    this->status = status_led_defs::messages::trial_off;
    this->sync_led_state = false;
    this->estop = false;
    
};

void ExoData::reconfigure(uint8_t* config_to_send) 
{
    left_leg.reconfigure(config_to_send);
    right_leg.reconfigure(config_to_send);
};

void ExoData::for_each_joint(for_each_joint_function_t function)
{
    function(&left_leg.hip);
    function(&left_leg.knee);
    function(&left_leg.ankle);
    function(&right_leg.hip);
    function(&right_leg.knee);
    function(&right_leg.ankle);
};


