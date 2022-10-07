/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Controller.h"
//#define CONTROLLER_DEBUG

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 
#include <math.h>

_Controller::_Controller(config_defs::joint_id id, ExoData* exo_data)
{
    _id = id;
    _data = exo_data;
    
    _t_helper = Time_Helper::get_instance();
    _t_helper_context = _t_helper->generate_new_context();
    _t_helper_delta_t = 0;
    
    // we just need to know the side to point at the right data location so it is only for the constructor
    bool is_left = utils::get_is_left(_id);
    #ifdef CONTROLLER_DEBUG
        Serial.print(is_left ? "Left " : "Right ");
    #endif 
    
    _integral_val = 0;
    _prev_input = 0; 
    _prev_de_dt = 0;
        
    // set _controller_data to point to the data specific to the controller.
    switch (utils::get_joint_type(_id))
    {
        case (uint8_t)config_defs::joint_id::hip:
            #ifdef CONTROLLER_DEBUG
                Serial.print("HIP ");
            #endif
            if (is_left)
            {
                _controller_data = &(exo_data->left_leg.hip.controller);
                _joint_data = &(exo_data->left_leg.hip);
            }
            else
            {
                _controller_data = &(exo_data->right_leg.hip.controller);
                _joint_data = &(exo_data->right_leg.hip);
            }
            break;
            
        case (uint8_t)config_defs::joint_id::knee:
            #ifdef CONTROLLER_DEBUG
                Serial.print("KNEE ");
            #endif
            if (is_left)
            {
                _controller_data = &(exo_data->left_leg.knee.controller);
                _joint_data = &(exo_data->left_leg.knee);
            }
            else
            {
                _controller_data = &(exo_data->right_leg.knee.controller);
                _joint_data = &(exo_data->right_leg.knee);
            }
            break;
        
        case (uint8_t)config_defs::joint_id::ankle:
            #ifdef CONTROLLER_DEBUG
                Serial.print("ANKLE ");
            #endif
            if (is_left)
            {
                _controller_data = &(exo_data->left_leg.ankle.controller);
                _joint_data = &(exo_data->left_leg.ankle);
            }
            else
            {
                _controller_data = &(exo_data->right_leg.ankle.controller);
                _joint_data = &(exo_data->right_leg.ankle);
            }
            break;
    }
    #ifdef CONTROLLER_DEBUG
        Serial.print("Controller : \n\t_controller_data set \n\t_joint_data set");
    #endif
    // added a pointer to the leg data as most controllers will need to access info specific to their leg.
    if (is_left)
    {
        _leg_data = &(exo_data->left_leg);
    }
    else
    {
        _leg_data = &(exo_data->right_leg);
    } 
    #ifdef CONTROLLER_DEBUG
        Serial.println("\n\t_leg_data set");
    #endif
    
};
 
float _Controller::_pid(float cmd, float measurement, float p_gain, float i_gain, float d_gain)
{ 
    //check if time is ok
    bool time_good = true;
    if (_t_helper->tick(_t_helper_context) > ((float) 1/LOOP_FREQ_HZ * 1000000 * (1 + LOOP_TIME_TOLERANCE)))
    {
        time_good = false;
    }

    //TODO: Try dynamic time step

    float error_val = cmd - measurement;  
    //_integral_val += error_val / LOOP_FREQ_HZ;     
    float de_dt = 0;
    if (time_good)
    {
       de_dt = -(measurement - _prev_input) * LOOP_FREQ_HZ; 
       _prev_de_dt = de_dt;
    }
    else 
    {
        //de_dt = _prev_de_dt;
        de_dt = 0;
    }
      
    _prev_input = measurement;
    float p = p_gain * error_val;  
    //float i = i_gain * _integral_val;  // resetting _integral_val was crashing the system 
    float d = d_gain * de_dt; 
    
    return p + d;
};

void _Controller::reset_integral()
{
    #ifdef CONTROLLER_DEBUG
        Serial.println("_Controller::reset_integral : Entered");
    #endif
    //this -> _integral_val = 0;
    
    #ifdef CONTROLLER_DEBUG
        Serial.println("_Controller::reset_integral : Exited");
    #endif
    
};

//****************************************************

ZeroTorque::ZeroTorque(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    
    #ifdef CONTROLLER_DEBUG
        Serial.println("ZeroTorque::Constructor");
    #endif
    
};

float ZeroTorque::calc_motor_cmd()
{
    float cmd_ff = 0;
    
    // add the PID contribution to the feed forward command
    float cmd = cmd_ff;
    if (_controller_data->parameters[controller_defs::zero_torque::use_pid_idx])
    {
        cmd = _pid(cmd_ff, _joint_data->torque_reading, _controller_data->parameters[controller_defs::zero_torque::p_gain_idx], _controller_data->parameters[controller_defs::zero_torque::i_gain_idx], _controller_data->parameters[controller_defs::zero_torque::d_gain_idx]);
    }
    _controller_data->ff_setpoint = cmd;

    // if (_joint_data->is_left) 
    // {
    //     Serial.print("ZeroTorque::calc_motor_cmd : torque_reading = ");
    //     Serial.print(_joint_data->torque_reading);
    //     Serial.print("\t");
    //     Serial.print("ZeroTorque::calc_motor_cmd : cmd = ");
    //     Serial.println(cmd);
    // }

    // Print PD gains
    // Serial.print("ZeroTorque::calc_motor_cmd : p_gain = ");
    // Serial.println(_controller_data->parameters[controller_defs::zero_torque::p_gain_idx]);
    // Serial.print("ZeroTorque::calc_motor_cmd : d_gain = ");
    // Serial.println(_controller_data->parameters[controller_defs::zero_torque::d_gain_idx]);
    return cmd;
};

//****************************************************

Stasis::Stasis(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    
    #ifdef CONTROLLER_DEBUG
        Serial.println("Stasis::Constructor");
    #endif
    
};

float Stasis::calc_motor_cmd()
{
    float cmd = 0;
    return cmd;
};


//****************************************************


ProportionalJointMoment::ProportionalJointMoment(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        Serial.println("ProportionalJointMoment::Constructor");
    #endif
    
};

float ProportionalJointMoment::calc_motor_cmd()
{
    static const float scaling_factor = 1;//2*18/4096;
    float cmd_ff = 0;
    static uint32_t run_count = 0;
    run_count++;
    bool print = false;
    if (run_count > 10)
    {
        run_count = 0;
        print = _leg_data->is_left;
    }


    // if (print)
    // {
    //     Serial.print("ProportionalJointMoment::calc_motor_cmd : Entered");
    //     Serial.print("\n");
    // }
    
    // don't calculate command when fsr is calibrating.
    if (!_leg_data->do_calibration_toe_fsr)
    {
        // calculate the feed forward command
        if (_leg_data->toe_stance) 
        {
            // scale the fsr values so the controller outputs zero feed forward when the FSR value is at the threshold
            float threshold = _leg_data->toe_fsr_upper_threshold;
            float scaling = threshold/(1-threshold);
            float correction = scaling-(scaling*_leg_data->toe_fsr);

            // saturate the fsr value
            float fsr = min(_leg_data->toe_fsr, 1.5);

            // calculate the feed forward command
            cmd_ff = (fsr) * _controller_data->parameters[controller_defs::proportional_joint_moment::stance_max_idx];
            // if (print)
            // {
            //     Serial.print("ProportionalJointMoment::calc_motor_cmd : cmd_ff before sat = ");
            //     Serial.println(cmd_ff);
            // }
            
            // saturate and account for assistance
            cmd_ff = min(max(0, cmd_ff), _controller_data->parameters[controller_defs::proportional_joint_moment::stance_max_idx]);
            cmd_ff = cmd_ff * (_controller_data->parameters[controller_defs::proportional_joint_moment::is_assitance_idx] ? -1 : 1);
            //cmd_ff = (_controller_data->parameters[controller_defs::proportional_joint_moment::is_assitance_idx] ? -1 : 1) * min(max(0, cmd_ff), _controller_data->parameters[controller_defs::proportional_joint_moment::stance_max_idx]);
            // if (print)
            // {
            //     Serial.print("ProportionalJointMoment::calc_motor_cmd : toe_stance");
            //     Serial.print(" fsr data: ");
            //     Serial.print(_leg_data->toe_fsr);
            //     Serial.print(" max stance: ");
            //     Serial.print(_controller_data->parameters[controller_defs::proportional_joint_moment::stance_max_idx]);
            //     Serial.print(" = ");
            //     Serial.println(cmd_ff);
            // }
        }
        else
        {
            cmd_ff = _controller_data->parameters[controller_defs::proportional_joint_moment::swing_max_idx];
        }
    }
    _controller_data->ff_setpoint = cmd_ff;

    // low pass filter on torque_reading (note the negative signs)
    _filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _filtered_torque_reading, 0.8);

    // TODO: Add auto kf to feed forward
    
    // add the PID contribution to the feed forward command
    float cmd;
    if (_controller_data->parameters[controller_defs::proportional_joint_moment::use_pid_idx])
    {

        cmd = _pid(cmd_ff, _filtered_torque_reading, _controller_data->parameters[controller_defs::proportional_joint_moment::p_gain_idx], _controller_data->parameters[controller_defs::proportional_joint_moment::i_gain_idx], _controller_data->parameters[controller_defs::proportional_joint_moment::d_gain_idx]);
    
    }
    else
    {
        cmd = cmd_ff;
    }
    
    if (print)
    {
        Serial.print("ProportionalJointMoment::calc_motor_cmd : torque reading = ");
        Serial.println(_joint_data->torque_reading);
        // Print PID gains if PID is used
        if (_controller_data->parameters[controller_defs::proportional_joint_moment::use_pid_idx])
        {
            Serial.print("ProportionalJointMoment::calc_motor_cmd : P = ");
            Serial.println(_controller_data->parameters[controller_defs::proportional_joint_moment::p_gain_idx]);
            Serial.print("ProportionalJointMoment::calc_motor_cmd : I = ");
            Serial.println(_controller_data->parameters[controller_defs::proportional_joint_moment::i_gain_idx]);
            Serial.print("ProportionalJointMoment::calc_motor_cmd : D = ");
            Serial.println(_controller_data->parameters[controller_defs::proportional_joint_moment::d_gain_idx]);
        }
        Serial.print("ProportionalJointMoment::calc_motor_cmd : cmd = ");
        Serial.print(cmd);
        Serial.print("\t");
        Serial.print("ProportionalJointMoment::calc_motor_cmd : cmd_ff = ");
        Serial.print(cmd_ff);
        Serial.print("\t");
        Serial.print("ProportionalJointMoment::calc_motor_cmd : filtered_torque_reading = ");
        Serial.println(_filtered_torque_reading * scaling_factor);
    }

    // if (print)
    // {
    //     Serial.print("ProportionalJointMoment::calc_motor_cmd : torque reading = ");
    //     Serial.println(_joint_data->torque_reading);
    //     // Print PID gains
    //     Serial.print("ProportionalJointMoment::calc_motor_cmd : P = ");
    //     Serial.println(_controller_data->parameters[controller_defs::proportional_joint_moment::p_gain_idx]);
    //     Serial.print("ProportionalJointMoment::calc_motor_cmd : I = ");
    //     Serial.println(_controller_data->parameters[controller_defs::proportional_joint_moment::i_gain_idx]);
    //     Serial.print("ProportionalJointMoment::calc_motor_cmd : D = ");
    //     Serial.println(_controller_data->parameters[controller_defs::proportional_joint_moment::d_gain_idx]);
    //     Serial.print("ProportionalJointMoment::calc_motor_cmd : cmd = ");
    //     Serial.println(cmd);
    //     Serial.print("ProportionalJointMoment::calc_motor_cmd : Exiting");
    //     Serial.print("\n");
    // }

    _filtered_cmd = utils::ewma(cmd, _filtered_cmd, 0.8);
    return _filtered_cmd * scaling_factor;
};


//****************************************************


HeelToe::HeelToe(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        Serial.println("HeelToe::Constructor");
    #endif
    
    
};

float HeelToe::calc_motor_cmd()
{
    // this code is just temporary while we are under construction.
    float cmd_ff = 0;
    
    // add the PID contribution to the feed forward command
    float cmd = cmd_ff + (_controller_data->parameters[controller_defs::heel_toe::use_pid_idx] 
                ? _pid(cmd_ff, _joint_data->torque_reading,_controller_data->parameters[controller_defs::heel_toe::p_gain_idx], _controller_data->parameters[controller_defs::heel_toe::i_gain_idx], _controller_data->parameters[controller_defs::heel_toe::d_gain_idx]) 
                : 0); 
                         
    return cmd;
};


//****************************************************


ExtensionAngle::ExtensionAngle(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        Serial.println("ExtensionAngle::Constructor");
    #endif
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
    
    float cmd_ff = 0;
    // calculate torque based on state
    switch (_state)
    {
        case 0 :  // extension
            cmd_ff = _controller_data->parameters[controller_defs::extension_angle::extension_setpoint_idx] * normalized_angle;
            break;
        case 1 :  // flexion
            cmd_ff = _controller_data->parameters[controller_defs::extension_angle::flexion_setpoint_idx];
            break;
    }
    
    // add the PID contribution to the feed forward command
    float cmd = cmd_ff + (_controller_data->parameters[controller_defs::extension_angle::use_pid_idx] 
                ? _pid(cmd_ff, _joint_data->torque_reading,_controller_data->parameters[controller_defs::extension_angle::p_gain_idx], _controller_data->parameters[controller_defs::extension_angle::i_gain_idx], _controller_data->parameters[controller_defs::extension_angle::d_gain_idx]) 
                : 0);
    
    return cmd;
};


void ExtensionAngle::_reset_angles()
{
    _max_angle = _initial_max_angle;
    _min_angle = _initial_min_angle;
};


void ExtensionAngle::_update_state(float angle)
{
    switch (_state)
    {
        case 0 :  // extension assistance
            if (angle <= _controller_data->parameters[controller_defs::extension_angle::angle_threshold_idx])           //If the angle of the hip is less than 5 degrees (currently what angle_threshold is set to), then switch to flexion assistance
            {
                _state = 1;
            }
            break;
        case 1 :  // flexion assistance 
            
            if ((angle > (_controller_data->parameters[controller_defs::extension_angle::target_flexion_percent_max_idx] * _max_angle / 100))           //If the angle exceeds 80% (currently what flexion_percent_max is set to) of the max angle, switch to extension angle
                || ((angle > _controller_data->parameters[controller_defs::extension_angle::angle_threshold_idx]+utils::degrees_to_radians(5))          //Or if the angle is greater than 10 degrees (angle_threshold currently set to 5 degrees), switch to extension 
                    && (_leg_data->hip.velocity <= _controller_data->parameters[controller_defs::extension_angle::velocity_threshold_idx])))            //or if the angular velocity is less than or equal to -0.175rad/s (currenlty what velocity_threshold is set to), switch to extension 
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


//****************************************************


BangBang::BangBang(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        Serial.println("BangBang::Constructor");
    #endif
    _state = 0; // extension mode originally 
    
    _reset_angles();
    
};

float BangBang::calc_motor_cmd()
{
    // check if the angle range should be reset
    if (_controller_data->parameters[controller_defs::bang_bang::clear_angle_idx])
    {
        _reset_angles();
    }
    
    float angle = _leg_data->hip.position;
    // check the angle range
    _max_angle = max(angle, _max_angle);
    _min_angle = min(angle, _min_angle);
    
    
    // float normalized_angle = 0;
    // // calculate the normalized angle
    // if (angle >= 0)
    // {
        // normalized_angle = angle / _max_angle;
    // }
    // else
    // {
        // normalized_angle = angle / _min_angle;
        
    // }

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
    
    float cmd_ff = 0;
    // calculate torque based on state
    switch (_state)
    {
        case 0 :  // extension
            cmd_ff = _controller_data->parameters[controller_defs::bang_bang::extension_setpoint_idx];
            break;
        case 1 :  // flexion
            cmd_ff = _controller_data->parameters[controller_defs::bang_bang::flexion_setpoint_idx];
            break;
    }
    
    
    // add the PID contribution to the feed forward command
    float cmd = cmd_ff + (_controller_data->parameters[controller_defs::bang_bang::use_pid_idx] 
                ? _pid(cmd_ff, _joint_data->torque_reading,_controller_data->parameters[controller_defs::bang_bang::p_gain_idx], _controller_data->parameters[controller_defs::bang_bang::i_gain_idx], _controller_data->parameters[controller_defs::bang_bang::d_gain_idx]) 
                : 0);
    
    return ((_controller_data->parameters[controller_defs::bang_bang::is_assitance_idx] ? 1:-1) * cmd);
};


void BangBang::_reset_angles()
{
    _max_angle = _initial_max_angle;
    _min_angle = _initial_min_angle;
};



void BangBang::_update_state(float angle)
{
    switch (_state)
    {
    case 0:  // extension assistance
        if (angle <= _controller_data->parameters[controller_defs::bang_bang::angle_threshold_idx])
        {
            _state = 1;
        }
        break;
    case 1:  // flexion assistance 

        if ((angle > (_controller_data->parameters[controller_defs::bang_bang::target_flexion_percent_max_idx] * _max_angle / 100))
            || ((angle > _controller_data->parameters[controller_defs::bang_bang::angle_threshold_idx] + utils::degrees_to_radians(5))
                && (_leg_data->hip.velocity <= _controller_data->parameters[controller_defs::bang_bang::velocity_threshold_idx])))
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
};


//****************************************************


LateStance::LateStance(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
#ifdef CONTROLLER_DEBUG
    Serial.println("LateStance::Constructor");
#endif
    _state = 0; // extension mode originally 

    _reset_angles();

};

float LateStance::calc_motor_cmd()
{
    // check if the angle range should be reset
    if (_controller_data->parameters[controller_defs::late_stance::clear_angle_idx])
    {
        _reset_angles();
    }

    float angle = _leg_data->hip.position;
    // check the angle range
    _max_angle = max(angle, _max_angle);
    _min_angle = min(angle, _min_angle);

    _update_state(angle);

    float cmd_ff = 0;
    // calculate torque based on state
    switch (_state)
    {
    case 0:  // extension
        cmd_ff = 0;
        break;
    case 1:  // flexion
        cmd_ff = _controller_data->parameters[controller_defs::late_stance::resistance_setpoint_idx];
        break;
    }


    float cmd = cmd_ff + (_controller_data->parameters[controller_defs::late_stance::use_pid_idx]
        ? _pid(cmd_ff, _joint_data->torque_reading, _controller_data->parameters[controller_defs::late_stance::p_gain_idx], _controller_data->parameters[controller_defs::late_stance::i_gain_idx], _controller_data->parameters[controller_defs::late_stance::d_gain_idx])
        : 0);

    return cmd;
};

/*
 * Used to reset the range of motion to the starting values.
 * There is no reset for the flag so the user must turn this off manually.
 */
void LateStance::_reset_angles()
{
    _max_angle = _initial_max_angle;
    _min_angle = _initial_min_angle;
};


/*
 *
 */
void LateStance::_update_state(float angle)
{
    switch (_state)
    {
    case 0:  // No Torque 
        if (angle <= _controller_data->parameters[controller_defs::late_stance::angle_on_off])
        {
            _state = 1;
        }
        break;
    case 1:  // Torque

        if ((angle > (_controller_data->parameters[controller_defs::late_stance::angle_on_off] * _max_angle / 100)))
        {
            _state = 0;
        }
        break;


    }
};


//****************************************************


GaitPhase::GaitPhase(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
#ifdef CONTROLLER_DEBUG
    Serial.println("GaitPhase::Constructor");
#endif
};

float GaitPhase::calc_motor_cmd()
{

    // Initializes torque
    float cmd_ff = 0;

    // Defines % Gait Variable (obtains from Leg.cpp & LegData.cpp)
    float percent_gait = _leg_data->percent_gait;

    //Print outs if you need to debug Controller parameter issue
    // 
    //Serial.print("GaitPhase::calc_motor_cmd : Flexion_Start_Percentage : ");
    //Serial.print(_controller_data->parameters[controller_defs::gait_phase::flexion_start_percentage_idx]);
    //Serial.print("\n");

    //Serial.print("GaitPhase::calc_motor_cmd : Flexion_End_Percentage : ");
    //Serial.print(_controller_data->parameters[controller_defs::gait_phase::flexion_end_percentage_idx]);
    //Serial.print("\n");

    //Serial.print("GaitPhase::calc_motor_cmd : Extension_Start_Percentage : ");
    //Serial.print(_controller_data->parameters[controller_defs::gait_phase::extension_start_percentage_idx]);
    //Serial.print("\n");

    //Serial.print("GaitPhase::calc_motor_cmd : Extension_End_Percentage : ");
    //Serial.print(_controller_data->parameters[controller_defs::gait_phase::extension_end_percentage_idx]);
    //Serial.print("\n");

    if (-1 != percent_gait) //Only runs if a valid calculation of percent gait is present
    {
        //If the percentage of gait is within the flexion start and end points, the motor supplies flexion assistance
        if ((percent_gait >= _controller_data->parameters[controller_defs::gait_phase::flexion_start_percentage_idx]) && (percent_gait <= _controller_data->parameters[controller_defs::gait_phase::flexion_end_percentage_idx]))
        {
            cmd_ff = _controller_data->parameters[controller_defs::gait_phase::flexion_setpoint_idx];
            //Serial.print("GaitPhase::calc_motor_cmd : Flexion : ");
            //Serial.print(percent_gait);
            //Serial.print("\n");

        }
        //If the percentage of gait is within the extension start and end points, the motor supplies extension assistance
        else if ((percent_gait > _controller_data->parameters[controller_defs::gait_phase::extension_start_percentage_idx]) && (percent_gait <= 100))
        {
            cmd_ff = _controller_data->parameters[controller_defs::gait_phase::extension_setpoint_idx];
            //Serial.print("GaitPhase::calc_motor_cmd : Extension - Late : ");
            //Serial.print(percent_gait);
            //Serial.print("\n");
        }
        //If the percentage is 0 or less than the flexion start point, the motor supplies extension assistance
        else if (percent_gait == 0 || percent_gait < _controller_data->parameters[controller_defs::gait_phase::extension_end_percentage_idx])
        {
            cmd_ff = _controller_data->parameters[controller_defs::gait_phase::extension_setpoint_idx];
            //Serial.print("GaitPhase::calc_motor_cmd : Extension - Early : ");
            //Serial.print(percent_gait);
            //Serial.print("\n");
        }
        //If none of these conditions are met, send a zero torque to the motor
        else 
        {
            cmd_ff = 0;
            //Serial.print("GaitPhase::calc_motor_cmd : Unknown Condition : ");
            //Serial.print(percent_gait);
            //Serial.print("\n");
        }
    }


    // Incorporates PID control if flag is present
    float cmd = cmd_ff + (_controller_data->parameters[controller_defs::gait_phase::use_pid_idx]
        ? _pid(cmd_ff, _joint_data->torque_reading, _controller_data->parameters[controller_defs::gait_phase::p_gain_idx], _controller_data->parameters[controller_defs::gait_phase::i_gain_idx], _controller_data->parameters[controller_defs::gait_phase::d_gain_idx])
        : 0);

    return cmd;
};


//****************************************************


/*
 * Constructor for the controller
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer.
 */
ZhangCollins::ZhangCollins(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        Serial.println("ZhangCollins::Constructor");
    #endif
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


void ZhangCollins::_update_spline_parameters(int mass, float peak_normalized_torque_Nm_kg, float ramp_start_percent_gait, float onset_percent_gait, float peak_percent_gait, float stop_percent_gait)
{
    
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
    
    // unnormalize the peak torque
    _tp_Nm = _mass * (float)peak_normalized_torque_Nm_kg;
    _ts_Nm = 2;

    // calculate the spline parameters
    _a1 = (2 *(_tp_Nm - _ts_Nm))/pow((t1 - t2),3);
    _b1 = -((3 *(t1 + t2) *(_tp_Nm - _ts_Nm)) / pow((t1 - t2),3));
    _c1 = (6* t1 * t2 * (_tp_Nm - _ts_Nm))/pow((t1 - t2),3);
    _d1 = -((-pow(t1, 3) * _tp_Nm + 3 * pow(t1, 2) * t2 * _tp_Nm - 3 * t1 * pow(t2,2) * _ts_Nm +
            pow(t2,3) * _ts_Nm)/pow((t1 - t2),3));

    
    _a2 = -((_tp_Nm - _ts_Nm)/(2* pow((t2 - t3),3)));
    _b2 = (3 *t3 *(_tp_Nm - _ts_Nm))/(2 *pow((t2 - t3),3));
    _c2 = (3 *(pow(t2,2) - 2 *t2 *t3) * (_tp_Nm - _ts_Nm))/(2* pow((t2 - t3),3));
    _d2 = -((3 * pow(t2,2) * t3 * _tp_Nm - 6 * t2 * pow(t3, 2) * _tp_Nm + 2 * pow(t3,3) * _tp_Nm -
              2 * pow(t2,3) * _ts_Nm + 3 * pow(t2, 2) * t3 * _ts_Nm)/(2 * pow((t2 - t3), 3)));


    return;
};

/*
 *
 */
float ZhangCollins::calc_motor_cmd()
{
    // check if the parameters have changed and update the spline if they have
    if ((_mass != _controller_data->parameters[controller_defs::zhang_collins::mass_idx])
        || (_peak_normalized_torque_Nm_kg != _controller_data->parameters[controller_defs::zhang_collins::peak_normalized_torque_Nm_kg_idx])
        || (_t0 != _controller_data->parameters[controller_defs::zhang_collins::t0_idx])
        || (_t1 != _controller_data->parameters[controller_defs::zhang_collins::t1_idx])
        || (_t2 != _controller_data->parameters[controller_defs::zhang_collins::t2_idx])
        || (_t3 != _controller_data->parameters[controller_defs::zhang_collins::t3_idx]))
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
    
    // add the PID contribution to the feed forward command
    float cmd = torque_cmd + (_controller_data->parameters[controller_defs::zhang_collins::use_pid_idx] 
                ? _pid(torque_cmd, _joint_data->torque_reading,_controller_data->parameters[controller_defs::zhang_collins::p_gain_idx], _controller_data->parameters[controller_defs::zhang_collins::i_gain_idx], _controller_data->parameters[controller_defs::zhang_collins::d_gain_idx]) 
                : 0); 
                
    return cmd;
};


//****************************************************


FranksCollinsHip::FranksCollinsHip(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        Serial.println("FranksCollinsHip::Constructor");
    #endif
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


void FranksCollinsHip::_update_spline_parameters(int mass,
float trough_normalized_torque_Nm_kg, float peak_normalized_torque_Nm_kg,
float start_percent_gait, float trough_onset_percent_gait, float trough_percent_gait,
float mid_percent_gait, float mid_duration_gait,
float peak_percent_gait, float peak_offset_percent_gait)
{
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
   
    // unnormalize torque
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


float FranksCollinsHip::calc_motor_cmd()
{
    // check if the parameters have changed and update the spline if they have
    if ((_mass != _controller_data->parameters[controller_defs::franks_collins_hip::mass_idx])
        || (_trough_normalized_torque_Nm_kg != _controller_data->parameters[controller_defs::franks_collins_hip::trough_normalized_torque_Nm_kg_idx])
        || (_peak_normalized_torque_Nm_kg != _controller_data->parameters[controller_defs::franks_collins_hip::peak_normalized_torque_Nm_kg_idx])
        || (_start_percent_gait != _controller_data->parameters[controller_defs::franks_collins_hip::start_percent_gait_idx])
        || (_trough_onset_percent_gait != _controller_data->parameters[controller_defs::franks_collins_hip::trough_onset_percent_gait_idx])
        || (_trough_percent_gait != _controller_data->parameters[controller_defs::franks_collins_hip::trough_percent_gait_idx])
        || (_mid_time != _controller_data->parameters[controller_defs::franks_collins_hip::mid_time_idx])
        || (_mid_duration != _controller_data->parameters[controller_defs::franks_collins_hip::mid_duration_idx])
        || (_peak_percent_gait != _controller_data->parameters[controller_defs::franks_collins_hip::peak_percent_gait_idx])
        || (_peak_offset_percent_gait != _controller_data->parameters[controller_defs::franks_collins_hip::peak_offset_percent_gait_idx]))
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
    
    // add the PID contribution to the feed forward command
    float cmd = torque_cmd + (_controller_data->parameters[controller_defs::franks_collins_hip::use_pid_idx] 
                ? _pid(torque_cmd, _joint_data->torque_reading,_controller_data->parameters[controller_defs::franks_collins_hip::p_gain_idx], _controller_data->parameters[controller_defs::franks_collins_hip::i_gain_idx], _controller_data->parameters[controller_defs::franks_collins_hip::d_gain_idx]) 
                : 0);  
                
    return cmd;
};


//****************************************************
// 
// 
// UserDefined::UserDefined(config_defs::joint_id id, ExoData* exo_data)
// : _Controller(id, exo_data)
// {
    // #ifdef CONTROLLER_DEBUG
        // Serial.println("UserDefined::Constructor");
    // #endif
    
    
    
    // for(int i = 0; i < controller_defs::user_defined::num_sample_points; i++) 
    // {
        // _percent_x[i] = i * _step_size;
    // } 
// };

// float UserDefined::calc_motor_cmd()
// {
    
    // int lower_idx = _leg_data->percent_gait/_step_size; //rounds down  
    // float cmd_ff = 0;
    // //  yp = y0 + ((y1-y0)/(x1-x0)) * (xp - x0);
    // if(lower_idx < controller_defs::user_defined::num_sample_points - 1)
    // {
        // cmd_ff = _controller_data->parameters[controller_defs::user_defined::curve_start_idx+lower_idx] + ((_controller_data->parameters[controller_defs::user_defined::curve_start_idx+lower_idx+1]-_controller_data->parameters[controller_defs::user_defined::curve_start_idx+lower_idx])/(_percent_x[lower_idx+1]-_percent_x[lower_idx])) * (_leg_data->percent_gait - _percent_x[lower_idx]);
    // } 
    // else
    // {
        // cmd_ff = _controller_data->parameters[controller_defs::user_defined::curve_start_idx+lower_idx] + ((_controller_data->parameters[0]-_controller_data->parameters[controller_defs::user_defined::curve_start_idx+lower_idx])/(100-_percent_x[lower_idx])) * (_leg_data->percent_gait - _percent_x[lower_idx]);
    // }
    // // add the PID contribution to the feed forward command
    // float cmd = cmd_ff + (_controller_data->parameters[controller_defs::user_defined::use_pid_idx] 
                // ? _pid(cmd_ff, _joint_data->torque_reading,_controller_data->parameters[controller_defs::user_defined::p_gain_idx], _controller_data->parameters[controller_defs::user_defined::i_gain_idx], _controller_data->parameters[controller_defs::user_defined::d_gain_idx]) 
                // : 0);
    
    // return 0;
// };


//****************************************************


Sine::Sine(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        Serial.println("Sine::Constructor");
    #endif
};

float Sine::calc_motor_cmd()
{
    //  converts period to int so % will work, but is only a float for convenience
    float cmd_ff = _controller_data->parameters[controller_defs::sine::amplitude_idx] * sin( (millis() % (int)_controller_data->parameters[controller_defs::sine::period_idx]) / _controller_data->parameters[controller_defs::sine::period_idx] * 2 * M_PI + _controller_data->parameters[controller_defs::sine::phase_shift_idx]  );
    
    // add the PID contribution to the feed forward command
    float cmd = cmd_ff + (_controller_data->parameters[controller_defs::sine::use_pid_idx] 
                ? _pid(cmd_ff, _joint_data->torque_reading,_controller_data->parameters[controller_defs::sine::p_gain_idx], _controller_data->parameters[controller_defs::sine::i_gain_idx], _controller_data->parameters[controller_defs::sine::d_gain_idx]) 
                : 0); 
    return cmd;
};
#endif