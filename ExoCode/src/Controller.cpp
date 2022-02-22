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
    // added a pointer to the leg data as most controllers will need to access info specific to their leg.
    if (is_left)
    {
        _leg_data = &(exo_data->left_leg);
    }
    else
    {
        _leg_data = &(exo_data->right_leg);
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
    //Serial.println("ProportionalJointMoment::calc_motor_cmd : Entered");
    cmd = _leg_data->toe_fsr * _controller_data->parameters[controller_defs::proportional_joint_moment::max_torque_idx];
    cmd = max(0, cmd);  // if the fsr is negative use zero torque so it doesn't dorsiflex.
    
    //Serial.println("ProportionalJointMoment::calc_motor_cmd : Exiting");
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
    // this code is just temporary while we are under construction.
    int cmd = 0;
    return cmd;
};


/*
 * Constructor for the controller
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer.
 */
ExtensionAngle::ExtensionAngle(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    _state = 0; // extension mode originally 
    
    _reset_angles();
    
};

int ExtensionAngle::calc_motor_cmd()
{
    // check if the angle range should be reset
    if (_controller_data->parameters[controller_defs::extension_angle::clear_angle_idx])
    {
        _reset_angles();
    }
    
    float angle = _leg_data->hip.motor.p;
    // check the angle range
    _max_angle = max(angle, _max_angle);
    _min_angle = min(angle, _min_angle);
    
    
    float normalized_angle = 0;
    // calculate the normalized angle
    if (angle >= 0)
    {
        normalized_angle = angle / _max_angle;
    }
    else
    {
        normalized_angle = angle / _min_angle;
        
    }
    // int print_time_ms = 100;
    // static int last_timestamp = millis();
    // int timestamp = millis();
    // if ((timestamp-last_timestamp)>print_time_ms)
    // {
        // Serial.print(utils::radians_to_degrees(angle));
        // Serial.print(" ");
        // Serial.print(utils::radians_to_degrees(_max_angle));
        // Serial.print(" ");
        // Serial.print(utils::radians_to_degrees(_min_angle));
        // Serial.print(" ");
        // Serial.print(100);
        // Serial.print(" ");
        // Serial.print(-100);
        // Serial.print(" ");
        // Serial.println(normalized_angle*100);
    // }
    
    _update_state(angle);
    
    int cmd = 0;
    // calculate torque based on state
    switch (_state)
    {
        case 0 :  // extension
            cmd = _controller_data->parameters[controller_defs::extension_angle::extension_setpoint_idx] * normalized_angle;
            break;
        case 1 :  // flexion
            cmd = _controller_data->parameters[controller_defs::extension_angle::flexion_setpoint_idx];
            break;
    }
    
    
    return cmd;
};

/*
 * Used to reset the range of motion to the starting values.
 * There is no reset for the flag so the user must turn this off manually.
 */
void ExtensionAngle::_reset_angles()
{
    _max_angle = _initial_max_angle;
    _min_angle = _initial_min_angle;
};


/*
 *
 */
void ExtensionAngle::_update_state(float angle)
{
    switch (_state)
    {
        case 0 :  // extension assistance
            if (angle <= 0)
            {
                _state = 1;
            }
            break;
        case 1 :  // flexion assistance 
            
            if ((angle > (_controller_data->parameters[controller_defs::extension_angle::target_flexion_percent_max_idx] * _max_angle / 100)) 
                | ((angle > utils::degrees_to_radians(5)) & (_leg_data->hip.motor.v <= 0)))
            {
                _state = 0;
            }
            break;
         
        
    }
    // int print_time_ms = 100;
    // static int last_timestamp = millis();
    // int timestamp = millis();
    // if ((timestamp-last_timestamp)>print_time_ms)
    // {
        // Serial.print(angle);
        // Serial.print(" ");
        // Serial.print(_controller_data->parameters[controller_defs::extension_angle::target_flexion_percent_max_idx] * _max_angle/100);
        // Serial.println(" ");
    // }
}


/*
 * Constructor for the controller
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer.
 */
 /*
ZhangCollins::ZhangCollins(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    _mass = -1;
    _peak_normalized_torque_x100 = -1;
    _t0_x10 = -1;
    _t1_x10 = -1;
    _t2_x10 = -1;
    _t3_x10 = -1;
            
    // peak torque
    _tp = -1;
    // cable tension torque.  Not needed for our design, but used to match the paper.
    _ts = -1;
    // parameters for rising spline
    _a1 = -1;
    _b1 = -1;
    _c1 = -1;
    _d1 = -1;
    
    // parameters for falling spline
    _a2 = -1;
    _b2 = -1;
    _c2 = -1;
    _d2 = -1;
};
*/



#endif