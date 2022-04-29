/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Controller.h"

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 
#include <math.h>
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

float ZeroTorque::calc_motor_cmd()
{
    float cmd = 0;
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

float ProportionalJointMoment::calc_motor_cmd()
{
    float cmd = 0;
    //Serial.print("ProportionalJointMoment::calc_motor_cmd : Entered");
    //Serial.print("\n");
    cmd = _leg_data->toe_fsr * _controller_data->parameters[controller_defs::proportional_joint_moment::max_torque_idx];
    cmd = max(0, cmd);  // if the fsr is negative use zero torque so it doesn't dorsiflex.
    
    //Serial.print("ProportionalJointMoment::calc_motor_cmd : Exiting");
    //Serial.print("\n");
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

float HeelToe::calc_motor_cmd()
{
    // this code is just temporary while we are under construction.
    float cmd = 0;
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

float ExtensionAngle::calc_motor_cmd()
{
    // check if the angle range should be reset
    if (_controller_data->parameters[controller_defs::extension_angle::clear_angle_idx])
    {
        _reset_angles();
    }
    
    float angle = _leg_data->hip.position;
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
    //     Serial.print(utils::radians_to_degrees(angle));
    //     Serial.print(" ");
    //     Serial.print(utils::radians_to_degrees(_max_angle));
    //     Serial.print(" ");
    //     Serial.print(utils::radians_to_degrees(_min_angle));
    //     Serial.print(" ");
    //     Serial.print(100);
    //     Serial.print(" ");
    //     Serial.print(-100);
    //     Serial.print(" ");
    //     Serial.print(normalized_angle*100);
    //     Serial.print("\n");
    // }
    
    _update_state(angle);
    
    float cmd = 0;
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
            if (angle <= _controller_data->parameters[controller_defs::extension_angle::angle_threshold_idx])
            {
                _state = 1;
            }
            break;
        case 1 :  // flexion assistance 
            
            if ((angle > (_controller_data->parameters[controller_defs::extension_angle::target_flexion_percent_max_idx] * _max_angle / 100)) 
                | ((angle > _controller_data->parameters[controller_defs::extension_angle::angle_threshold_idx]+utils::degrees_to_radians(5)) 
                    & (_leg_data->hip.velocity <= _controller_data->parameters[controller_defs::extension_angle::velocity_threshold_idx])))
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
        // Serial.print("\n");
    // }
}

/*
 * Constructor for the controller
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer.
 */
BangBang::BangBang(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    _state = 0; // extension mode originally 
    
    _reset_angles();
    
};

float BangBang::calc_motor_cmd()
{
    // check if the angle range should be reset
    if (_controller_data->parameters[controller_defs::extension_angle::clear_angle_idx])
    {
        _reset_angles();
    }
    
    float angle = _leg_data->hip.position;
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
    //     Serial.print(utils::radians_to_degrees(angle));
    //     Serial.print(" ");
    //     Serial.print(utils::radians_to_degrees(_max_angle));
    //     Serial.print(" ");
    //     Serial.print(utils::radians_to_degrees(_min_angle));
    //     Serial.print(" ");
    //     Serial.print(100);
    //     Serial.print(" ");
    //     Serial.print(-100);
    //     Serial.print(" ");
    //     Serial.print(normalized_angle*100);
    //     Serial.print("\n");
    // }
    
    _update_state(angle);
    
    float cmd = 0;
    // calculate torque based on state
    switch (_state)
    {
        case 0 :  // extension
            cmd = _controller_data->parameters[controller_defs::extension_angle::extension_setpoint_idx];
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
void BangBang::_reset_angles()
{
    _max_angle = _initial_max_angle;
    _min_angle = _initial_min_angle;
};


/*
 *
 */
void BangBang::_update_state(float angle)
{
    switch (_state)
    {
        case 0 :  // extension assistance
            if (angle <= _controller_data->parameters[controller_defs::extension_angle::angle_threshold_idx])
            {
                _state = 1;
            }
            break;
        case 1 :  // flexion assistance 
            
            if ((angle > (_controller_data->parameters[controller_defs::extension_angle::target_flexion_percent_max_idx] * _max_angle / 100)) 
                | ((angle > _controller_data->parameters[controller_defs::extension_angle::angle_threshold_idx]+utils::degrees_to_radians(5)) 
                    & (_leg_data->hip.velocity <= _controller_data->parameters[controller_defs::extension_angle::velocity_threshold_idx])))
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
        // Serial.print("\n");
    // }
}


/*
 * Constructor for the controller
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer.
 */
ZhangCollins::ZhangCollins(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    _mass = -1;
    _peak_normalized_torque_Nm_kg = -1;
    _t0 = -1;
    _t1 = -1;
    _t2 = -1;
    _t3 = -1;
            
    // peak torque
    _tp_Nm = -1;
    // cable tension torque.  Not needed for our design, but used to match the paper.
    _ts_Nm = -1;
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

/*
 * Used to update the parameters of the spline equations
 * when the parameters that define the splines change.
 * 
 * Takes in the parameters that define the curve shape.
 */
void ZhangCollins::_update_spline_parameters(int mass, float peak_normalized_torque_Nm_kg, float ramp_start_percent_gait, float onset_percent_gait, float peak_percent_gait, float stop_percent_gait)
{
    // TODO: add config file read;
    // 1 cout << "\n exoBoot :: initCollinsProfile : Doing the init." << endl;

    _mass = mass; // kg
    _t0 = ramp_start_percent_gait;
    _t1 = onset_percent_gait;
    _t2 = peak_percent_gait;
    _t3 = stop_percent_gait;
    
    // todo : add fixed point
    float t0 = ramp_start_percent_gait;
    float t1 = onset_percent_gait;
    float t2 = peak_percent_gait;
    float t3 = stop_percent_gait;
    
    
    
    

    _peak_normalized_torque_Nm_kg = peak_normalized_torque_Nm_kg; // 0.76; // Using a smaller value due to Dephy Exo Limit.
    

    _tp_Nm = _mass * (float)peak_normalized_torque_Nm_kg;
    _ts_Nm = 2;

    _a1 = (2 *(_tp_Nm - _ts_Nm))/pow((t1 - t2),3);
    _b1 = -((3 *(t1 + t2) *(_tp_Nm - _ts_Nm)) / pow((t1 - t2),3));
    _c1 = (6* t1 * t2 * (_tp_Nm - _ts_Nm))/pow((t1 - t2),3);
    _d1 = -((-pow(t1, 3) * _tp_Nm + 3 * pow(t1, 2) * t2 * _tp_Nm - 3 * t1 * pow(t2,2) * _ts_Nm +
            pow(t2,3) * _ts_Nm)/pow((t1 - t2),3));

    // 1 cout << "exoBoot :: initCollinsProfile : \na1 = " << a1 << "\nb1 = " << b1 << "\nc1 = " << c1 << "\nd1 = " << d1 << endl;

    _a2 = -((_tp_Nm - _ts_Nm)/(2* pow((t2 - t3),3)));
    _b2 = (3 *t3 *(_tp_Nm - _ts_Nm))/(2 *pow((t2 - t3),3));
    _c2 = (3 *(pow(t2,2) - 2 *t2 *t3) * (_tp_Nm - _ts_Nm))/(2* pow((t2 - t3),3));
    _d2 = -((3 * pow(t2,2) * t3 * _tp_Nm - 6 * t2 * pow(t3, 2) * _tp_Nm + 2 * pow(t3,3) * _tp_Nm -
              2 * pow(t2,3) * _ts_Nm + 3 * pow(t2, 2) * t3 * _ts_Nm)/(2 * pow((t2 - t3), 3)));

    // 1 cout << "exoBoot :: initCollinsProfile : \na2 = " << a2 << "\nb2 = " << b2 << "\nc2 = " << c2 << "\nd2 = " << d2 << endl;


    // 1 cout << "\n\nexoBoot :: initCollinsProfile : \ntorque rising = " << a1 << " t^3 + " << b1 << " t^2 + " << c1 << " t + " << d1 << endl;
    // 1 cout << "exoBoot :: initCollinsProfile : \ntorque falling = " << a2 << " t^3 + " << b2 << " t^2 + " << c2 << " t + " << d2 << endl;


    return;
};

/*
 *
 */
float ZhangCollins::calc_motor_cmd()
{
    // check if the parameters have changed and update the spline if they have
    if ((_mass != _controller_data->parameters[controller_defs::zhang_collins::mass_idx])
        | (_peak_normalized_torque_Nm_kg != _controller_data->parameters[controller_defs::zhang_collins::peak_normalized_torque_Nm_kg_idx])
        | (_t0 != _controller_data->parameters[controller_defs::zhang_collins::t0_idx])
        | (_t1 != _controller_data->parameters[controller_defs::zhang_collins::t1_idx])
        | (_t2 != _controller_data->parameters[controller_defs::zhang_collins::t2_idx])
        | (_t3 != _controller_data->parameters[controller_defs::zhang_collins::t3_idx]))
    {
        _update_spline_parameters(_controller_data->parameters[controller_defs::zhang_collins::mass_idx]
            , _controller_data->parameters[controller_defs::zhang_collins::peak_normalized_torque_Nm_kg_idx]
            , _controller_data->parameters[controller_defs::zhang_collins::t0_idx]
            , _controller_data->parameters[controller_defs::zhang_collins::t1_idx]
            , _controller_data->parameters[controller_defs::zhang_collins::t2_idx]
            , _controller_data->parameters[controller_defs::zhang_collins::t3_idx]);
        // Serial.print("ZhangCollins::calc_motor_cmd : Updated parameters");
        // Serial.print("\n");
        // delay(1000);
    
    }
    
    // based on the percent gait find the torque to apply.
    // convert to floats so we don't need to modify everywhere in the code.
    float percent_gait = _leg_data->percent_gait;
    float t0 = _t0;
    float t1 = _t1;
    float t2 = _t2;
    float t3 = _t3;
    float torque_cmd = 0;
    
    // Serial.print("ZhangCollins::calc_motor_cmd : Percent Gait = ");
    // Serial.print(percent_gait);
    // Serial.print("\n");
    // Serial.print("ZhangCollins::calc_motor_cmd : t0 = ");
    // Serial.print(t0);
    // Serial.print("\n");
    // Serial.print("ZhangCollins::calc_motor_cmd : t1 = ");
    // Serial.print(t1);
    // Serial.print("\n");
    // Serial.print("ZhangCollins::calc_motor_cmd : t2 = ");
    // Serial.print(t2);
    //Serial.print("\n");
    // Serial.print("ZhangCollins::calc_motor_cmd : t3 = ");
    // Serial.print(t3);
    //Serial.print("\n\n");
    
    
    
    if (-1 != percent_gait) //we have a valid calculation for percent_gait
    {
        if ((percent_gait <= t1) && (0 <= percent_gait))  // torque ramp to ts at t1
        {
            torque_cmd = _ts_Nm / (t1 - t0) * percent_gait - _ts_Nm/(t1 - t0) * t0;  
            // Serial.print("ZhangCollins::calc_motor_cmd : Ramp");
            // Serial.print("\n");
            
        }
        else if (percent_gait <= t2) // the rising spline
        {
            torque_cmd = _a1 * pow(percent_gait,3) + _b1 * pow(percent_gait,2) + _c1 * percent_gait + _d1;
            // Serial.print("ZhangCollins::calc_motor_cmd : Rising");
            // Serial.print("\n");
        }
        else if (percent_gait <= t3)  // the falling spline
        {
            torque_cmd = _a2 * pow(percent_gait,3) + _b2 * pow(percent_gait,2) + _c2 * percent_gait + _d2;
            // Serial.print("ZhangCollins::calc_motor_cmd : Falling");
            // Serial.print("\n");
        }
        else  // go to the slack position if we aren't commanding a specific value
        {
            torque_cmd = 0;
            // Serial.print("ZhangCollins::calc_motor_cmd : Swing");
            // Serial.print("\n");
        }
    }
    return torque_cmd;
};



FranksCollinsHip::FranksCollinsHip(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    _last_start_time = -1;
    _last_percent_gait = -1;
    _mass = -1;
    _mid_time = -1;
    _mid_duration = -1;
   
    _trough_onset_percent_gait = -1;
    _trough_percent_gait = -1;
       
    _peak_percent_gait = -1;
    _peak_offset_percent_gait = -1;
   
    _trough_normalized_torque_Nm_kg = -1;
    _t0_trough = -1;
    _t1_trough = -1;
    _t2_trough = -1;
    _t3_trough = -1;
           
    // trough torque
    _tt_Nm = -1;
   
    // parameters for rising spline
    _a1_trough = -1;
    _b1_trough = -1;
    _c1_trough = -1;
    _d1_trough = -1;
   
    // parameters for falling spline
    _a2_trough = -1;
    _b2_trough = -1;
    _c2_trough = -1;
    _d2_trough = -1;
   
    _peak_normalized_torque_Nm_kg = -1;
    _t0_peak = -1;
    _t1_peak = -1;
    _t2_peak = -1;
    _t3_peak = -1;
           
    // peak torque
    _tp_Nm = -1;
   
    // parameters for rising spline
    _a1_peak = -1;
    _b1_peak = -1;
    _c1_peak = -1;
    _d1_peak = -1;
   
    // parameters for falling spline
    _a2_peak = -1;
    _b2_peak = -1;
    _c2_peak = -1;
    _d2_peak = -1;
};


/*
 * Used to update the parameters of the spline equations
 * when the parameters that define the splines change.
 *
 * Takes in the parameters that define the curve shape.
 */

void FranksCollinsHip::_update_spline_parameters(int mass,
float trough_normalized_torque_Nm_kg, float peak_normalized_torque_Nm_kg,
float start_percent_gait, float trough_onset_percent_gait, float trough_percent_gait,
float mid_percent_gait, float mid_duration_gait,
float peak_percent_gait, float peak_offset_percent_gait)
{
    // TODO: add config file read;
    // 1 cout << "\n exoBoot :: initCollinsProfile : Doing the init." << endl;
    // Serial.print("Franks::_update_spline_parameters : mass = ");
    // Serial.print(mass);
    // Serial.print("\n");
   
    _mass = mass; // kg
   
    _mid_time = mid_percent_gait;
    _mid_duration = mid_duration_gait;
   
    _trough_onset_percent_gait = trough_onset_percent_gait;
    _trough_percent_gait = trough_percent_gait;
       
    _peak_percent_gait = peak_percent_gait;
    _peak_offset_percent_gait = peak_offset_percent_gait;
   
    // shift whole curve to start at start_percent_gait
    _start_percent_gait = start_percent_gait;
    _t0_trough = 0;
    _t1_trough = trough_onset_percent_gait - _start_percent_gait < 0 ? trough_onset_percent_gait - _start_percent_gait + 100 : trough_onset_percent_gait - _start_percent_gait;
    _t2_trough = trough_percent_gait - _start_percent_gait < 0 ? trough_percent_gait - _start_percent_gait + 100 : trough_percent_gait - _start_percent_gait;
    _t3_trough = _mid_time - (_mid_duration/2) - _start_percent_gait < 0 ? _mid_time - (_mid_duration/2) - _start_percent_gait + 100 : _mid_time - (_mid_duration/2) - _start_percent_gait;
   
    _t0_peak = _mid_time - _start_percent_gait < 0 ? _mid_time - _start_percent_gait + 100 : _mid_time - _start_percent_gait;
    _t1_peak = _mid_time + (_mid_duration/2) - _start_percent_gait < 0 ? _mid_time + (_mid_duration/2) - _start_percent_gait + 100 : _mid_time + (_mid_duration/2) - _start_percent_gait;
    _t2_peak = peak_percent_gait - _start_percent_gait < 0 ? peak_percent_gait - _start_percent_gait + 100 : peak_percent_gait - _start_percent_gait;
    _t3_peak = peak_offset_percent_gait - _start_percent_gait < 0 ? peak_offset_percent_gait - _start_percent_gait + 100 : peak_offset_percent_gait - _start_percent_gait;
   
       
    // todo : add fixed point
    // float t0 = ramp_start_percent_gait;
    // float t1 = onset_percent_gait;
    // float t2 = peak_percent_gait;
    // float t3 = stop_percent_gait;
   
   
    _trough_normalized_torque_Nm_kg = trough_normalized_torque_Nm_kg; // 0.76; // Using a smaller value due to Dephy Exo Limit.
    _peak_normalized_torque_Nm_kg = peak_normalized_torque_Nm_kg; // 0.76; // Using a smaller value due to Dephy Exo Limit.
   

    _tt_Nm = _mass * _trough_normalized_torque_Nm_kg;
    _tp_Nm = _mass * _peak_normalized_torque_Nm_kg;
    _tts_Nm = 0; // Negative
    _tps_Nm = 0; //Positive

    _a1_trough = (2 *(_tt_Nm - _tts_Nm))/pow((_t1_trough - _t2_trough),3);
    _b1_trough = -((3 *(_t1_trough + _t2_trough) *(_tt_Nm - _tts_Nm)) / pow((_t1_trough - _t2_trough),3));
    _c1_trough = (6* _t1_trough * _t2_trough * (_tt_Nm - _tts_Nm))/pow((_t1_trough - _t2_trough),3);
    _d1_trough = -((-pow(_t1_trough, 3) * _tt_Nm + 3 * pow(_t1_trough, 2) * _t2_trough * _tt_Nm - 3 * _t1_trough * pow(_t2_trough,2) * _tts_Nm +
            pow(_t2_trough,3) * _tts_Nm)/pow((_t1_trough - _t2_trough),3));

    _a2_trough = -((_tt_Nm - _tts_Nm)/(2* pow((_t2_trough - _t3_trough),3)));
    _b2_trough = (3 * _t3_trough *(_tt_Nm - _tts_Nm))/(2 *pow((_t2_trough - _t3_trough),3));
    _c2_trough = (3 *(pow(_t2_trough,2) - 2 * _t2_trough * _t3_trough) * (_tt_Nm - _tts_Nm))/(2* pow((_t2_trough - _t3_trough),3));
    _d2_trough = -((3 * pow(_t2_trough,2) * _t3_trough * _tt_Nm - 6 * _t2_trough * pow(_t3_trough, 2) * _tt_Nm + 2 * pow(_t3_trough,3) * _tt_Nm -
              2 * pow(_t2_trough,3) * _tts_Nm + 3 * pow(_t2_trough, 2) * _t3_trough * _tts_Nm)/(2 * pow((_t2_trough - _t3_trough), 3)));

    _a1_peak = (2 *(_tp_Nm - _tps_Nm))/pow((_t1_peak - _t2_peak),3);
    _b1_peak = -((3 *(_t1_peak + _t2_peak) *(_tp_Nm - _tps_Nm)) / pow((_t1_peak - _t2_peak),3));
    _c1_peak = (6* _t1_peak * _t2_peak * (_tp_Nm - _tps_Nm))/pow((_t1_peak - _t2_peak),3);
    _d1_peak = -((-pow(_t1_peak, 3) * _tp_Nm + 3 * pow(_t1_peak, 2) * _t2_peak * _tp_Nm - 3 * _t1_peak * pow(_t2_peak,2) * _tps_Nm +
            pow(_t2_peak,3) * _tps_Nm)/pow((_t1_peak - _t2_peak),3));

    _a2_peak = -((_tp_Nm - _tps_Nm)/(2* pow((_t2_peak - _t3_peak),3)));
    _b2_peak = (3 * _t3_peak *(_tp_Nm - _tps_Nm))/(2 *pow((_t2_peak - _t3_peak),3));
    _c2_peak = (3 *(pow(_t2_peak,2) - 2 * _t2_peak * _t3_peak) * (_tp_Nm - _tps_Nm))/(2* pow((_t2_peak - _t3_peak),3));
    _d2_peak = -((3 * pow(_t2_peak,2) * _t3_peak * _tp_Nm - 6 * _t2_peak * pow(_t3_peak, 2) * _tp_Nm + 2 * pow(_t3_peak,3) * _tp_Nm -
              2 * pow(_t2_peak,3) * _tps_Nm + 3 * pow(_t2_peak, 2) * _t3_peak * _tps_Nm)/(2 * pow((_t2_peak - _t3_peak), 3)));


    return;
};


/*
 *
 */

float FranksCollinsHip::calc_motor_cmd()
{
    // check if the parameters have changed and update the spline if they have
    if ((_mass != _controller_data->parameters[controller_defs::franks_collins_hip::mass_idx])
        | (_trough_normalized_torque_Nm_kg != _controller_data->parameters[controller_defs::franks_collins_hip::trough_normalized_torque_Nm_kg_idx])
        | (_peak_normalized_torque_Nm_kg != _controller_data->parameters[controller_defs::franks_collins_hip::peak_normalized_torque_Nm_kg_idx])
        | (_start_percent_gait != _controller_data->parameters[controller_defs::franks_collins_hip::start_percent_gait_idx])
        | (_trough_onset_percent_gait != _controller_data->parameters[controller_defs::franks_collins_hip::trough_onset_percent_gait_idx])
        | (_trough_percent_gait != _controller_data->parameters[controller_defs::franks_collins_hip::trough_percent_gait_idx])
        | (_mid_time != _controller_data->parameters[controller_defs::franks_collins_hip::mid_time_idx])
        | (_mid_duration != _controller_data->parameters[controller_defs::franks_collins_hip::mid_duration_idx])
        | (_peak_percent_gait != _controller_data->parameters[controller_defs::franks_collins_hip::peak_percent_gait_idx])
        | (_peak_offset_percent_gait != _controller_data->parameters[controller_defs::franks_collins_hip::peak_offset_percent_gait_idx]))
    {
        _update_spline_parameters(_controller_data->parameters[controller_defs::franks_collins_hip::mass_idx]
            , _controller_data->parameters[controller_defs::franks_collins_hip::trough_normalized_torque_Nm_kg_idx]
            , _controller_data->parameters[controller_defs::franks_collins_hip::peak_normalized_torque_Nm_kg_idx]
            , _controller_data->parameters[controller_defs::franks_collins_hip::start_percent_gait_idx]
            , _controller_data->parameters[controller_defs::franks_collins_hip::trough_onset_percent_gait_idx]
            , _controller_data->parameters[controller_defs::franks_collins_hip::trough_percent_gait_idx]
            , _controller_data->parameters[controller_defs::franks_collins_hip::mid_time_idx]
            , _controller_data->parameters[controller_defs::franks_collins_hip::mid_duration_idx]
            , _controller_data->parameters[controller_defs::franks_collins_hip::peak_percent_gait_idx]
            , _controller_data->parameters[controller_defs::franks_collins_hip::peak_offset_percent_gait_idx]);
           
           
        // Serial.print("Franks::calc_motor_cmd : Updated parameters");
        // Serial.print("\n");
       
        // Serial.print("mass :\t\t\t");
        // Serial.print(_mass);
        // Serial.print("\t");
        // Serial.print(_controller_data->parameters[controller_defs::franks_collins_hip::mass_idx]);
        // Serial.print("\n");
       
        // Serial.print("_trough_normalized_torque_Nm_kg :\t");
        // Serial.print(_trough_normalized_torque_Nm_kg);
        // Serial.print("\t");
        // Serial.print(_controller_data->parameters[controller_defs::franks_collins_hip::trough_normalized_torque_Nm_kg_idx]);
        // Serial.print("\n");
       
        // Serial.print("_peak_normalized_torque_Nm_kg :\t");
        // Serial.print(_peak_normalized_torque_Nm_kg);
        // Serial.print("\t");
        // Serial.print(_controller_data->parameters[controller_defs::franks_collins_hip::peak_normalized_torque_Nm_kg_idx]);
        // Serial.print("\n");
       
        // Serial.print("_start_percent_gait :\t\t\t");
        // Serial.print(_start_percent_gait );
        // Serial.print("\t");
        // Serial.print(_controller_data->parameters[controller_defs::franks_collins_hip::start_percent_gait_idx]);
        // Serial.print("\n");
       
        // Serial.print("_trough_onset_percent_gait :\t\t\t");
        // Serial.print(_trough_onset_percent_gait);
        // Serial.print("\t");
        // Serial.print(_controller_data->parameters[controller_defs::franks_collins_hip::trough_onset_percent_gait_idx]);
        // Serial.print("\n");
       
        // Serial.print("_trough_percent_gait :\t\t\t");
        // Serial.print(_trough_percent_gait);
        // Serial.print("\t");
        // Serial.print(_controller_data->parameters[controller_defs::franks_collins_hip::trough_percent_gait_idx]);
        // Serial.print("\n");
       
        // Serial.print("_mid_time :\t\t\t");
        // Serial.print(_mid_time );
        // Serial.print("\t");
        // Serial.print(_controller_data->parameters[controller_defs::franks_collins_hip::mid_time_idx]);
        // Serial.print("\n");
       
        // Serial.print("_mid_duration :\t\t\t");
        // Serial.print(_mid_duration );
        // Serial.print("\t");
        // Serial.print(_controller_data->parameters[controller_defs::franks_collins_hip::mid_duration_idx]);
        // Serial.print("\n");
       
        // Serial.print("_peak_percent_gait :\t\t\t");
        // Serial.print(_peak_percent_gait );
        // Serial.print("\t");
        // Serial.print(_controller_data->parameters[controller_defs::franks_collins_hip::peak_percent_gait_idx]);
        // Serial.print("\n");
       
        // Serial.print("_peak_offset_percent_gait :\t\t\t");
        // Serial.print(_peak_offset_percent_gait );
        // Serial.print("\t");
        // Serial.print(_controller_data->parameters[controller_defs::franks_collins_hip::peak_offset_percent_gait_idx]);
        // Serial.print("\n\n\n");
        // delay(1000);
   
    }
   
    // based on the percent gait find the torque to apply.
    // convert to floats so we don't need to modify everywhere in the code.
    float percent_gait = _leg_data->percent_gait;
    float expected_duration = _leg_data->expected_step_duration;
   
    // calculate the offset percent gait so there isn't a hold at 100% regular if the step goes long or short.
    // restart when the percent gait crosses the _start_percent_gait
    if ((percent_gait >= _start_percent_gait) && _last_percent_gait <_start_percent_gait)
    {
        _last_start_time = millis();
    }
   
    // Serial.print("Franks::calc_motor_cmd : _last_start_time = ");
    // Serial.print(_last_start_time);
    // Serial.print("\n");
    // Serial.print("Franks::calc_motor_cmd : percent_gait = ");
    // Serial.print(percent_gait);
    // Serial.print("\n");
    // Serial.print("Franks::calc_motor_cmd : _last_percent_gait = ");
    // Serial.print(_last_percent_gait);
    // Serial.print("\n");
   
    _last_percent_gait = percent_gait;
   
   
    // return 0 torque till we hit the correct percent gait.
    if (_last_start_time == -1)
    {
        return 0;
    }
   
    float shifted_percent_gait = (millis()-_last_start_time) / expected_duration * 100;
   
    // Serial.print("Franks::calc_motor_cmd : shifted_percent_gait = ");
    // Serial.print(shifted_percent_gait);
    // Serial.print("\n");
   
    float torque_cmd = 0;
   
    // Serial.print("FranksCollinsHip::calc_motor_cmd : shifted_percent_gait = ");
    // Serial.print(shifted_percent_gait);
    // Serial.print("\n");
    // Serial.print("FranksCollinsHip::calc_motor_cmd : _t0_trough = ");
    // Serial.print(_t0_trough);
    // Serial.print("\n");
    // Serial.print("FranksCollinsHip::calc_motor_cmd : _t1_trough = ");
    // Serial.print(_t1_trough);
    // Serial.print("\n");
    // Serial.print("FranksCollinsHip::calc_motor_cmd : _t2_trough = ");
    // Serial.print(_t2_trough);
    // Serial.print("\n");
    // Serial.print("FranksCollinsHip::calc_motor_cmd : _t3_trough = ");
    // Serial.print(_t3_trough);
    // Serial.print("\n");
    // Serial.print("FranksCollinsHip::calc_motor_cmd : _t1_peak = ");
    // Serial.print(_t1_peak);
    // Serial.print("\n");
    // Serial.print("FranksCollinsHip::calc_motor_cmd : _t2_peak = ");
    // Serial.print(_t2_peak);
    // Serial.print("\n");
    // Serial.print("FranksCollinsHip::calc_motor_cmd : _t3_peak = ");
    // Serial.print(_t3_peak);
    // Serial.print("\n");
    // Serial.print("FranksCollinsHip::calc_motor_cmd : _peak_offset_percent_gait = ");
    // Serial.print(_peak_offset_percent_gait);
    // Serial.print("\n\n");
   
   
   
    if ((shifted_percent_gait <= _t1_trough) && (_t0_trough <= shifted_percent_gait))  // torque ramp to ts at t1
    {
        torque_cmd = _tts_Nm / (_t1_trough - _t0_trough) * shifted_percent_gait - _tts_Nm/(_t1_trough - _t0_trough) * _t0_trough;  
        // Serial.print("FranksCollinsHip::calc_motor_cmd : Ramp");
        // Serial.print("\n");
       
    }
    else if (shifted_percent_gait <= _t2_trough) // the falling trough
    {
        torque_cmd = _a1_trough * pow(shifted_percent_gait,3) + _b1_trough * pow(shifted_percent_gait,2) + _c1_trough * shifted_percent_gait + _d1_trough;
        // Serial.print("FranksCollinsHip::calc_motor_cmd : falling trough");
        // Serial.print("\n");
    }
    else if (shifted_percent_gait <= _t3_trough)  // the rising trough
    {
        torque_cmd = _a2_trough * pow(shifted_percent_gait,3) + _b2_trough * pow(shifted_percent_gait,2) + _c2_trough * shifted_percent_gait + _d2_trough;
        // Serial.print("FranksCollinsHip::calc_motor_cmd : rising trough");
        // Serial.print("\n");
    }
    else if (shifted_percent_gait <= _t1_peak) // mid period
    {
        torque_cmd = _tps_Nm / (_t1_peak - _t3_trough) * shifted_percent_gait - _tps_Nm/(_t1_peak - _t3_trough) * _t3_trough;  
        // Serial.print("FranksCollinsHip::calc_motor_cmd : Ramp");
        // Serial.print("\n");  
    }
    else if (shifted_percent_gait <= _t2_peak) // the rising peak
    {
        torque_cmd = _a1_peak * pow(shifted_percent_gait,3) + _b1_peak * pow(shifted_percent_gait,2) + _c1_peak * shifted_percent_gait + _d1_peak;
        // Serial.print("FranksCollinsHip::calc_motor_cmd : Rising peak");
        // Serial.print("\n");
    }
    else if (shifted_percent_gait <= _t3_peak)  // the falling peak
    {
        torque_cmd = _a2_peak * pow(shifted_percent_gait,3) + _b2_peak * pow(shifted_percent_gait,2) + _c2_peak * shifted_percent_gait + _d2_peak;
        // Serial.print("FranksCollinsHip::calc_motor_cmd : Falling peak");
        // Serial.print("\n");
    }
    else  // go to the slack position if we aren't commanding a specific value
    {
        torque_cmd = 0;
        // Serial.print("FranksCollinsHip::calc_motor_cmd : end of cycle");
        // Serial.print("\n");
    }
   
   
    // Serial.print("FranksCollinsHip::calc_motor_cmd : torque_cmd = ");
    // Serial.print(torque_cmd);
    // Serial.print("\n\n");
    return torque_cmd;
};

#endif