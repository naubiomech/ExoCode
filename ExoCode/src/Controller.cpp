/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Controller.h"

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36) 
/*
 * Constructor for the controller
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer.
 */
Controller::Controller(config_defs::joint_id id, ExoData* exo_data)
{
    this->id = id;
    this->_data = exo_data;
};
#endif