/*
 * 
 * P. Stegall Jan. 2022
*/

#include "MotorData.h"
#include "ParseIni.h"

/*
 * Constructor for the motor data.
 * Takes the joint id and the array from the INI parser.
 * Stores the id, if it is on the left side (for convenience), and the motor type
 * It also has the info for the motor CAN packages.
 * TODO: decide what other data to store.
 */
MotorData::MotorData(config_defs::joint_id id, uint8_t* config_to_send)
{
    this->id = id;
    this->is_left = ((uint8_t)this->id & (uint8_t)config_defs::joint_id::left) == (uint8_t)config_defs::joint_id::left;
     
    
    switch ((uint8_t)this->id & (~(uint8_t)config_defs::joint_id::left & ~(uint8_t)config_defs::joint_id::right))  // use the id with the side masked out.
    {
        case (uint8_t)config_defs::joint_id::hip:
        {
            motor_type = config_to_send[config_defs::hip_idx];
            
            switch (config_to_send[config_defs::hip_gear_idx])
            {
                case (uint8_t)config_defs::gearing::gearing_1_1:
                {
                    gearing = 1;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_2_1:
                {
                    gearing = 2;
                    break;
                }                
                case (uint8_t)config_defs::gearing::gearing_3_1:
                {
                    gearing = 3;
                    break;
                }
                default:
                {
                    gearing = 1;
                    break;
                }
            }  
            if ((config_to_send[config_defs::hip_flip_dir_idx] == (uint8_t)config_defs::flip_dir::both) || ((config_to_send[config_defs::hip_flip_dir_idx] == (uint8_t)config_defs::flip_dir::left) && this->is_left) || ((config_to_send[config_defs::hip_flip_dir_idx] == (uint8_t)config_defs::flip_dir::right) && (!this->is_left)))
            {
                this->flip_direction = 1;
            }
            else
            {
                this->flip_direction = 0;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::knee:
        {
            motor_type = config_to_send[config_defs::knee_idx];
            
            switch (config_to_send[config_defs::knee_gear_idx])
            {
                case (uint8_t)config_defs::gearing::gearing_1_1:
                {
                    gearing = 1;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_2_1:
                {
                    gearing = 2;
                    break;
                } 
                case (uint8_t)config_defs::gearing::gearing_3_1:
                {
                    gearing = 3;
                    break;
                }
                default:
                {
                    gearing = 1;
                    break;
                }
            }  
            
            if ((config_to_send[config_defs::knee_flip_dir_idx] == (uint8_t)config_defs::flip_dir::both) || ((config_to_send[config_defs::knee_flip_dir_idx] == (uint8_t)config_defs::flip_dir::left) && this->is_left) || ((config_to_send[config_defs::knee_flip_dir_idx] == (uint8_t)config_defs::flip_dir::right) && (!this->is_left)))
            {
                this->flip_direction = 1;
            }
            else
            {
                this->flip_direction = 0;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::ankle:
        {
            motor_type = config_to_send[config_defs::ankle_idx];
            
            switch (config_to_send[config_defs::ankle_gear_idx])
            {
                case (uint8_t)config_defs::gearing::gearing_1_1:
                {
                    gearing = 1;
                    break;
                }
                case (uint8_t)config_defs::gearing::gearing_2_1:
                {
                    gearing = 2;
                    break;
                } 
                case (uint8_t)config_defs::gearing::gearing_3_1:
                {
                    gearing = 3;
                    break;
                }
                default:
                {
                    gearing = 1;
                    break;
                }
            }  
            
            if ((config_to_send[config_defs::ankle_flip_dir_idx] == (uint8_t)config_defs::flip_dir::both) || ((config_to_send[config_defs::ankle_flip_dir_idx] == (uint8_t)config_defs::flip_dir::left) && this->is_left) || ((config_to_send[config_defs::ankle_flip_dir_idx] == (uint8_t)config_defs::flip_dir::right) && (!this->is_left)))
            {
                this->flip_direction = 1;
            }
            else
            {
                this->flip_direction = 0;
            }
            break;
        }
    }
    
    
    // this is only setup for AK motors at the moment,  I think we will need to add other motor parameters for different types and just change the ones that are used rather than trying to just have motor specific parameters.
    p = 0; // read position
    v = 0; // read velocity
    i = 0; // read current
    p_des = 0; // 
    v_des = 0;
    kp = 0;
    kd = 0;
    t_ff = 0;
    
    
};

