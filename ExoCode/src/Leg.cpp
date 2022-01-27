/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Leg.h"

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36) 
/*
 * Constructor for the leg
 * Takes in if the leg is the left one and a pointer to the exo_data
 * Uses initializer list for hip, knee, and ankle joint; and the FSRs.
 * Only stores these objects, exo_data pointer, and if it is left (for easy access)
 */
Leg::Leg(bool is_left, ExoData* exo_data)
: _hip((config_defs::joint_id)((uint8_t)(is_left ? config_defs::joint_id::left : config_defs::joint_id::right) | (uint8_t)config_defs::joint_id::hip), exo_data)  // we need to cast to uint8_t to do bitwise or, then we have to cast it back to joint_id
, _knee((config_defs::joint_id)((uint8_t)(is_left ? config_defs::joint_id::left : config_defs::joint_id::right) | (uint8_t)config_defs::joint_id::knee), exo_data)
, _ankle((config_defs::joint_id)((uint8_t)(is_left ? config_defs::joint_id::left : config_defs::joint_id::right) | (uint8_t)config_defs::joint_id::ankle), exo_data)
, _heel_fsr(is_left ? logic_micro_pins::fsr_sense_left_heel_pin : logic_micro_pins::fsr_sense_right_heel_pin) // Check if it is the left and use the appropriate pin for the side.
, _toe_fsr(is_left ? logic_micro_pins::fsr_sense_left_toe_pin : logic_micro_pins::fsr_sense_right_toe_pin)
{
    this->_data = exo_data;
    this->_is_left = is_left;
};
#endif