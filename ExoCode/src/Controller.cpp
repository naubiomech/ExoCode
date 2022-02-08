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
ZeroTorque::ZeroTorque(config_defs::joint_id id, ExoData* exo_data)
{
    this->_id = id;
    this->_data = exo_data;
    
    
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
{
    this->_id = id;
    this->_data = exo_data;
    
    
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
{
    this->_id = id;
    this->_data = exo_data;
    
    
};

int HeelToe::calc_motor_cmd()
{
    int cmd = 0;
    return cmd;
};






#endif