/*
 * 
 * P. Stegall Jan. 2022
*/

#include "ControllerData.h"

/*
 * Constructor for the controller data.
 * Takes the joint id and the array from the INI parser.
 * Stores the id, sets the controller to the default controller for the appropriate joint, and records the joint type to check we are using appropriate controllers.
 * TODO: decide what other data to store.  I created a setpoint and a couple of generic parameters as place holders. 
 */
ControllerData::ControllerData(config_defs::joint_id id, uint8_t* config_to_send)
{
    
    switch ((uint8_t)id & (~(uint8_t)config_defs::joint_id::left & ~(uint8_t)config_defs::joint_id::right))  // use the id with the side masked out.
    {
        case (uint8_t)config_defs::joint_id::hip:
        {
            controller = config_to_send[config_defs::exo_hip_default_controller_idx];
            joint = config_defs::JointType::hip;
            break;
        }
        case (uint8_t)config_defs::joint_id::knee:
        {
            controller = config_to_send[config_defs::exo_knee_default_controller_idx];
            joint = config_defs::JointType::knee;
            break;
        }
        case (uint8_t)config_defs::joint_id::ankle:
        {
            controller = config_to_send[config_defs::exo_ankle_default_controller_idx];
            joint = config_defs::JointType::ankle;
            break;
        }
    }
    
    setpoint = 0;
    for (int i=0; i < controller_defs::max_parameters; i++)
    {    
        parameters[i] = 0;
    }
};

void ControllerData::reconfigure(uint8_t* config_to_send) 
{
    // just reset controller
    switch ((uint8_t)joint)  // use the id with the side masked out.
    {
        case (uint8_t)config_defs::joint_id::hip:
        {
            controller = config_to_send[config_defs::exo_hip_default_controller_idx];
            break;
        }
        case (uint8_t)config_defs::joint_id::knee:
        {
            controller = config_to_send[config_defs::exo_knee_default_controller_idx];
            break;
        }
        case (uint8_t)config_defs::joint_id::ankle:
        {
            controller = config_to_send[config_defs::exo_ankle_default_controller_idx];
            break;
        }
    }
    
    setpoint = 0;
    for (int i=0; i < controller_defs::max_parameters; i++)
    {    
        parameters[i] = 0;
    }
};

