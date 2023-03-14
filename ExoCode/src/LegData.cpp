#include "LegData.h"
#include "Logger.h"
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
    
    this->percent_gait = -1; // likely want to do fixed point 
    this->expected_step_duration = -1; 
    this->heel_fsr = -1; // set to -1 since should always be positive once set.
    this->toe_fsr = -1;
    this->do_calibration_toe_fsr = false; 
    this->do_calibration_refinement_toe_fsr = false; 
    this->do_calibration_heel_fsr = false; 
    this->do_calibration_refinement_heel_fsr = false; 
    this->ground_strike = false; 
    this->toe_off = false;
    
    this->expected_duration_window_upper_coeff = 1.75;
    this->expected_duration_window_lower_coeff = 0.25;

    this->inclination = Inclination::Level;

    this->PHJM_state = 0;

    // check if the leg is used from the config.
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

void LegData::reconfigure(uint8_t* config_to_send) 
{
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
    
    // reconfigure the joints
    hip.reconfigure(config_to_send);
    knee.reconfigure(config_to_send);
    ankle.reconfigure(config_to_send);
};
 