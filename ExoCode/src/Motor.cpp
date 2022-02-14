/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Motor.h"

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36) 


_Motor::_Motor(config_defs::joint_id id, ExoData* exo_data)
{
    _id = id;
    _is_left = ((uint8_t)this->_id & (uint8_t)config_defs::joint_id::left) == (uint8_t)config_defs::joint_id::left;
    _data = exo_data;
    
    // set _motor_data to point to the data specific to this motor.
    switch (utils::get_joint_type(_id))
    {
        case (uint8_t)config_defs::joint_id::hip:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_leg.hip.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_leg.hip.motor);
            }
            break;
            
        case (uint8_t)config_defs::joint_id::knee:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_leg.knee.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_leg.knee.motor);
            }
            break;
        
        case (uint8_t)config_defs::joint_id::ankle:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_leg.ankle.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_leg.ankle.motor);
            }
            break;
    }
    
};

bool _Motor::get_is_left() // constructor: type is the motor type
{
    return _is_left;
};

config_defs::joint_id _Motor::get_id()
{
    return _id;
};


/*
 * Constructor for the CAN Motor.  
 * We are using multilevel inheritance, so we have a general motor type, which is inherited by the CAN (e.g. TMotor) or other type (e.g. Maxon) since models within these types will share communication protocols, which is then inherited by the specific motor model (e.g. AK60), which may have specific torque constants etc.
 * 
 * 
 */
_CANMotor::_CANMotor(config_defs::joint_id id, ExoData* exo_data) // constructor: type is the motor type
: _Motor(id, exo_data)
{
    
};

void _CANMotor::read_data()
{
    // Read date from motor
    
    // set data in ExoData object
};

void _CANMotor::send_data()
{
    // Read date from ExoData object
    
    // set data in motor
};

void _CANMotor::on_off(bool is_on)
{
    if (is_on)
    {
        //enable motor
    }
    else 
    {
        //disable motor
    }
}

//**************************************
/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK60::AK60(config_defs::joint_id id, ExoData* exo_data): // constructor: type is the motor type
_CANMotor(id, exo_data)
{
    
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK80::AK80(config_defs::joint_id id, ExoData* exo_data): // constructor: type is the motor type
_CANMotor(id, exo_data)
{
    
};




#endif