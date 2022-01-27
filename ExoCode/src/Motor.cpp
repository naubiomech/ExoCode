/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Motor.h"

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36) 
/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
Motor::Motor(config_defs::joint_id id, ExoData* exo_data) // constructor: type is the motor type
{
    this->id = id;
    this->_is_left = ((uint8_t)this->id & (uint8_t)config_defs::joint_id::left) == (uint8_t)config_defs::joint_id::left;
    
    this->_data = exo_data;
};
#endif