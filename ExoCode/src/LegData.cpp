/*
 * 
 * P. Stegall Jan. 2022
*/

#include "LegData.h"



/*
 * Constructor for the leg data.
 * Takes if the leg is left, and the array from the INI parser.
 * Stores if the leg is left, the percent gait, FSR data, and if the leg is used.
 * Uses an initializer list for the joint data, and creates the joint id for the joints. 
 */
LegData::LegData(bool is_left, uint8_t* config_to_send)
: hip((config_defs::joint_id)((uint8_t)(is_left ? config_defs::joint_id::left : config_defs::joint_id::right) | (uint8_t)config_defs::joint_id::hip), config_to_send)
, knee((config_defs::joint_id)((uint8_t)(is_left ? config_defs::joint_id::left : config_defs::joint_id::right) | (uint8_t)config_defs::joint_id::knee), config_to_send)
, ankle((config_defs::joint_id)((uint8_t)(is_left ? config_defs::joint_id::left : config_defs::joint_id::right) | (uint8_t)config_defs::joint_id::ankle), config_to_send) 
{
    this->is_left = is_left;
    
    this->percent_gait_x10 = -1; // likely want to do fixed point 
    this->heel_fsr = -1;
    this->toe_fsr = -1;
    this->do_calibration_toe_fsr = false; //bit 0 is calibrate fsr, bit 1 is refine calibration.
    this->do_calibration_refinement_toe_fsr = false; 
    this->do_calibration_heel_fsr = false; //bit 0 is calibrate fsr, bit 1 is refine calibration.
    this->do_calibration_refinement_heel_fsr = false; 
    this->ground_strike = false; 

    if ((static_cast<uint8_t>(config_defs::exo_side::bilateral) == config_to_send[config_defs::exo_side_idx]) 
        || (((uint8_t)config_defs::exo_side::left == config_to_send[config_defs::exo_side_idx]) & this->is_left)
        || (((uint8_t)config_defs::exo_side::right == config_to_send[config_defs::exo_side_idx]) & !this->is_left)
       )
    {
        this->is_used = true;
    }
    else
    {
        this->is_used = false;
    }
        
};
 