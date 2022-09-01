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
    
    this->position = 0;
    this->velocity = 0;
    this->calibrate_torque_sensor = 0;
    
    switch ((uint8_t)this->id & (~(uint8_t)config_defs::joint_id::left & ~(uint8_t)config_defs::joint_id::right))  // use the id with the side masked out.
    {
        case (uint8_t)config_defs::joint_id::hip:
        {
            // Check if joint and side is used
            is_used = (config_to_send[config_defs::hip_idx] != (uint8_t)config_defs::motor::not_used) && ((static_cast<uint8_t>(config_defs::exo_side::bilateral) == config_to_send[config_defs::exo_side_idx]) 
                || (((uint8_t)config_defs::exo_side::left == config_to_send[config_defs::exo_side_idx]) && this->is_left)
                || (((uint8_t)config_defs::exo_side::right == config_to_send[config_defs::exo_side_idx]) && !this->is_left));
            // Serial.print("Hip\n");
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
            // Check if joint and side is used
            is_used = (config_to_send[config_defs::knee_idx] != (uint8_t)config_defs::motor::not_used) && ((static_cast<uint8_t>(config_defs::exo_side::bilateral) == config_to_send[config_defs::exo_side_idx]) 
                || (((uint8_t)config_defs::exo_side::left == config_to_send[config_defs::exo_side_idx]) && this->is_left)
                || (((uint8_t)config_defs::exo_side::right == config_to_send[config_defs::exo_side_idx]) && !this->is_left));
            // Serial.print("Knee\n");
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
            // Check if joint and side is used
            is_used = (config_to_send[config_defs::ankle_idx] != (uint8_t)config_defs::motor::not_used) && ((static_cast<uint8_t>(config_defs::exo_side::bilateral) == config_to_send[config_defs::exo_side_idx]) 
                || (((uint8_t)config_defs::exo_side::left == config_to_send[config_defs::exo_side_idx]) && this->is_left)
                || (((uint8_t)config_defs::exo_side::right == config_to_send[config_defs::exo_side_idx]) && !this->is_left));
            // Serial.print("Ankle\n");
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
       
};


void JointData::reconfigure(uint8_t* config_to_send) 
{
    switch ((uint8_t)this->id & (~(uint8_t)config_defs::joint_id::left & ~(uint8_t)config_defs::joint_id::right))  // use the id with the side masked out.
    {
        case (uint8_t)config_defs::joint_id::hip:
        {
            // Check if joint and side is used
            is_used = (config_to_send[config_defs::hip_idx] != (uint8_t)config_defs::motor::not_used) && ((static_cast<uint8_t>(config_defs::exo_side::bilateral) == config_to_send[config_defs::exo_side_idx]) 
                || (((uint8_t)config_defs::exo_side::left == config_to_send[config_defs::exo_side_idx]) && this->is_left)
                || (((uint8_t)config_defs::exo_side::right == config_to_send[config_defs::exo_side_idx]) && !this->is_left));
            // Serial.print("Hip\n");
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
            // Check if joint and side is used
            is_used = (config_to_send[config_defs::knee_idx] != (uint8_t)config_defs::motor::not_used) && ((static_cast<uint8_t>(config_defs::exo_side::bilateral) == config_to_send[config_defs::exo_side_idx]) 
                || (((uint8_t)config_defs::exo_side::left == config_to_send[config_defs::exo_side_idx]) && this->is_left)
                || (((uint8_t)config_defs::exo_side::right == config_to_send[config_defs::exo_side_idx]) && !this->is_left));
            // Serial.print("Knee\n");
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
            // Check if joint and side is used
            is_used = (config_to_send[config_defs::ankle_idx] != (uint8_t)config_defs::motor::not_used) && ((static_cast<uint8_t>(config_defs::exo_side::bilateral) == config_to_send[config_defs::exo_side_idx]) 
                || (((uint8_t)config_defs::exo_side::left == config_to_send[config_defs::exo_side_idx]) && this->is_left)
                || (((uint8_t)config_defs::exo_side::right == config_to_send[config_defs::exo_side_idx]) && !this->is_left));
            // Serial.print("Ankle\n");
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
    
    motor.reconfigure(config_to_send);
    controller.reconfigure(config_to_send);
};

