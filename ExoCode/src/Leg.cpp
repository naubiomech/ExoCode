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
    _data = exo_data;
    _is_left = is_left;
    _leg_data = _is_left ? &(_data->left_leg) : &(_data->right_leg);
    
    _last_heel_contact_state = true; // initialized to true so we don't get a strike the first time we read
    _last_toe_contact_state = true;
};

void Leg::read_data()
{
    _leg_data->heel_fsr = _heel_fsr.read();
    _leg_data->toe_fsr = _toe_fsr.read();
    _leg_data->ground_strike = _check_ground_strike();
    
};

void Leg::check_calibration()
{
    if (_leg_data->is_used)
    {
        // make sure calibration is done before refinement.
        if (_leg_data->do_calibration_toe_fsr)
        {
            _leg_data->do_calibration_toe_fsr = _toe_fsr.calibrate(_leg_data->do_calibration_toe_fsr);
        }
        else if (_leg_data->do_calibration_refinement_toe_fsr) 
        {
            _leg_data->do_calibration_refinement_toe_fsr = _toe_fsr.refine_calibration(_leg_data->do_calibration_refinement_toe_fsr);
        }
        
        if (_leg_data->do_calibration_heel_fsr)
        {
            _leg_data->do_calibration_heel_fsr = _heel_fsr.calibrate(_leg_data->do_calibration_heel_fsr);
        }
        else if (_leg_data->do_calibration_refinement_heel_fsr) 
        {
            _leg_data->do_calibration_refinement_heel_fsr = _heel_fsr.refine_calibration(_leg_data->do_calibration_refinement_heel_fsr);
        }
    }        
};

bool Leg::_check_ground_strike()
{
    bool heel_contact_state = _heel_fsr.get_ground_contact();
    bool toe_contact_state = _toe_fsr.get_ground_contact();
    bool ground_strike = false;
    
    // only check if in swing
    if(!_last_heel_contact_state & !_last_toe_contact_state)
    {
        //check for rising edge on heel and toe, toe is to account for flat foot landings
        if ((heel_contact_state > _last_heel_contact_state) | (toe_contact_state > _last_toe_contact_state))
        {
            ground_strike = true;
        }
    }
    _last_heel_contact_state = heel_contact_state;
    _last_toe_contact_state = toe_contact_state;
    
    return ground_strike;
};
#endif