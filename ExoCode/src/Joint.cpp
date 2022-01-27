/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Joint.h"

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36) 
// initialize the used joint counters that will be used to select the TorqueSensor pin.  If you don't do it it won't work.
uint8_t Joint::left_used_joint_count = 0;
uint8_t Joint::right_used_joint_count = 0;

/*
 * Constructor for the joint
 * Takes the joint id and a pointer to the exo_data
 * Uses initializer list for motor, controller, and torque sensor.
 * Only stores these objects, the id, exo_data pointer, and if it is left (for easy access)
 */
Joint::Joint(config_defs::joint_id id, ExoData* exo_data)
: _motor(id, exo_data)
, _controller(id, exo_data)
, _torque_sensor(Joint::get_torque_sensor_pin(id, exo_data))
{
    this->id = id;
    this->is_left = Joint::get_is_left(this->id); //((uint8_t)this->id & (uint8_t)config_defs::joint_id::left) == (uint8_t)config_defs::joint_id::left;
    
    this->_data = exo_data;
};  

/*
 * Takes in the joint id and returns if the left indicator bit is set as a bool
 *
 */
bool Joint::get_is_left(config_defs::joint_id id)
{
    return ((uint8_t)id & (uint8_t)config_defs::joint_id::left) == (uint8_t)config_defs::joint_id::left;
};

/*
 * Takes in the joint id and returns the id with the left/right bits masked out.
 * Returning uint8_t rather than joint_id type since we have to typecast to do logical stuff anyways.
 */
uint8_t Joint::get_joint_type(config_defs::joint_id id)
{
    return (uint8_t)id & (~(uint8_t)config_defs::joint_id::left & ~(uint8_t)config_defs::joint_id::right);  // return the joint id with the left/right indicators masked out.  
};

/*
 * Takes in the joint id and exo data, and checks if the current joint is used.
 * If it is used it pulls the next open torque sensor pin for the side, and increments the counter.
 * If the joint is not used, or we have used up all the available torque sensor pins for the side, it sets the pin to a pin that is not connected.
 */
unsigned int Joint::get_torque_sensor_pin(config_defs::joint_id id, ExoData* exo_data)
{
    // First check which joint we are looking at.  
    // Then go through and if it is the left or right and if it is used.  
    // If it is set return the appropriate pin and increment the counter.
    switch (Joint::get_joint_type(id))
    {
        case (uint8_t)config_defs::joint_id::hip:
        {
            if (Joint::get_is_left(id) & exo_data->left_leg.hip.is_used)  // check if the left leg is used
            {
                if (Joint::left_used_joint_count < logic_micro_pins::num_available_joints) // if we still have available pins send the next one and increment the counter.  If we don't send the not connected pin.
                {
                    return logic_micro_pins::torque_sensor_left[Joint::left_used_joint_count++];
                }
                else
                {
                    return logic_micro_pins::not_connected_pin;
                }
            }
            else if (exo_data->right_leg.hip.is_used)  // check if the right leg is used
            {
                if (Joint::right_used_joint_count < logic_micro_pins::num_available_joints) // if we still have available pins send the next one and increment the counter.  If we don't send the not connected pin.
                {
                    return logic_micro_pins::torque_sensor_right[Joint::right_used_joint_count++];
                }
                else
                {
                    return logic_micro_pins::not_connected_pin;
                }
            }
            else  // the joint isn't used.  I didn't optimize for the minimal number of logical checks because this should just be used at startup.
            {
                return logic_micro_pins::not_connected_pin;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::knee:
        {
            if (Joint::get_is_left(id) & exo_data->left_leg.knee.is_used)  // check if the left leg is used
            {
                if (Joint::left_used_joint_count < logic_micro_pins::num_available_joints) // if we still have available pins send the next one and increment the counter.  If we don't send the not connected pin.
                {
                    return logic_micro_pins::torque_sensor_left[Joint::left_used_joint_count++];
                }
                else
                {
                    return logic_micro_pins::not_connected_pin;
                }
            }
            else if (exo_data->right_leg.knee.is_used)  // check if the right leg is used
            {
                if (Joint::right_used_joint_count < logic_micro_pins::num_available_joints) // if we still have available pins send the next one and increment the counter.  If we don't send the not connected pin.
                {
                    return logic_micro_pins::torque_sensor_right[Joint::right_used_joint_count++];
                }
                else
                {
                    return logic_micro_pins::not_connected_pin;
                }
            }
            else  // the joint isn't used.  I didn't optimize for the minimal number of logical checks because this should just be used at startup.
            {
                return logic_micro_pins::not_connected_pin;
            }
            break;
        }
        case (uint8_t)config_defs::joint_id::ankle:
        {
            if (Joint::get_is_left(id) & exo_data->left_leg.ankle.is_used)  // check if the left leg is used
            {
                if (Joint::left_used_joint_count < logic_micro_pins::num_available_joints) // if we still have available pins send the next one and increment the counter.  If we don't send the not connected pin.
                {
                    return logic_micro_pins::torque_sensor_left[Joint::left_used_joint_count++];
                }
                else
                {
                    return logic_micro_pins::not_connected_pin;
                }
            }
            else if (exo_data->right_leg.ankle.is_used)  // check if the right leg is used
            {
                if (Joint::right_used_joint_count < logic_micro_pins::num_available_joints) // if we still have available pins send the next one and increment the counter.  If we don't send the not connected pin.
                {
                    return logic_micro_pins::torque_sensor_right[Joint::right_used_joint_count++];
                }
                else
                {
                    return logic_micro_pins::not_connected_pin;
                }
            }
            else  // the joint isn't used.  I didn't optimize for the minimal number of logical checks because this should just be used at startup.
            {
                return logic_micro_pins::not_connected_pin;
            }
            break;
        }
        default :
        {
            return logic_micro_pins::not_connected_pin;
        }
    }
};

#endif