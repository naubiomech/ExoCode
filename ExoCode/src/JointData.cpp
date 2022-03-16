/*
 * 
 * P. Stegall Jan. 2022
*/

#include "JointData.h"

/*
 * Constructor for the Joint data.
 * Takes the joint id and the array from the INI parser.
 * Stores the id, torque sensor reading, if it is on the left side (for convenience), and if the joint is used.
 * Uses an initializer list for the motor and controller data. 
 */
JointData::JointData(config_defs::joint_id id, uint8_t* config_to_send)
: motor(id, config_to_send)
, controller(id, config_to_send)
{
    
    this->id = id;
    
    this->torque_reading = 0;
    this->is_left = ((uint8_t)this->id & (uint8_t)config_defs::joint_id::left) == (uint8_t)config_defs::joint_id::left;
    
    switch ((uint8_t)this->id & (~(uint8_t)config_defs::joint_id::left & ~(uint8_t)config_defs::joint_id::right))  // use the id with the side masked out.
    {
        case (uint8_t)config_defs::joint_id::hip:
        {
            is_used = config_to_send[config_defs::hip_idx] != (uint8_t)config_defs::motor::not_used;
            // Serial.print("Hip\n");
            break;
        }
        case (uint8_t)config_defs::joint_id::knee:
        {
            is_used = config_to_send[config_defs::knee_idx] != (uint8_t)config_defs::motor::not_used;
            // Serial.print("Knee\n");
            break;
        }
        case (uint8_t)config_defs::joint_id::ankle:
        {
            is_used = config_to_send[config_defs::ankle_idx] != (uint8_t)config_defs::motor::not_used;
            // Serial.print("Ankle\n");
            break;
        }
    }
       
};


