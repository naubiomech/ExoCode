/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Controller.h"

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36) 

//****************************************************
/*
 * Constructor for the controller
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer.
 */
_Controller::_Controller(config_defs::joint_id id, ExoData* exo_data)
{
    _id = id;
    _data = exo_data;
    
    // we just need to know the side to point at the right data location so it is only for the constructor
    bool is_left = utils::get_is_left(_id);
    
    // set _controller_data to point to the data specific to the controller.
    switch (utils::get_joint_type(_id))
    {
        case (uint8_t)config_defs::joint_id::hip:
            if (is_left)
            {
                _controller_data = &(exo_data->left_leg.hip.controller);
            }
            else
            {
                _controller_data = &(exo_data->right_leg.hip.controller);
            }
            break;
            
        case (uint8_t)config_defs::joint_id::knee:
            if (is_left)
            {
                _controller_data = &(exo_data->left_leg.knee.controller);
            }
            else
            {
                _controller_data = &(exo_data->right_leg.knee.controller);
            }
            break;
        
        case (uint8_t)config_defs::joint_id::ankle:
            if (is_left)
            {
                _controller_data = &(exo_data->left_leg.ankle.controller);
            }
            else
            {
                _controller_data = &(exo_data->right_leg.ankle.controller);
            }
            break;
    }
};


//****************************************************
/*
 * Constructor for the controller
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer.
 */
ZeroTorque::ZeroTorque(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    
    
    
};

int ZeroTorque::calc_motor_cmd()
{
    int cmd = 0;
    return cmd;
};


//****************************************************
/*
 * Constructor for the controller
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer.
 */
ProportionalJointMoment::ProportionalJointMoment(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    
    
    
};

int ProportionalJointMoment::calc_motor_cmd()
{
    int cmd = 0;
    return cmd;
};


/*
 * Constructor for the controller
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer.
 */
HeelToe::HeelToe(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    
    
    
};

int HeelToe::calc_motor_cmd()
{
    int cmd = 0;
    return cmd;
};






#endif