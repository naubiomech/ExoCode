/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Leg.h"

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36) 
/*
 * Constructor for the leg
 * Takes in if the leg is the left one and a pointer to the exo_data
 * Uses initializer list for hip, knee, and ankle joint; and the FSRs.
 * Only stores these objects, exo_data pointer, and if it is left (for easy access)
 */
Leg::Leg(bool is_left, ExoData* exo_data)
: _hip((config_defs::joint_id)((uint8_t)(is_left ? config_defs::joint_id::left : config_defs::joint_id::right) | (uint8_t)config_defs::joint_id::hip), exo_data)  // we need to cast to uint8_t to do bitwise or, then we have to cast it back to joint_id
, _knee((config_defs::joint_id)((uint8_t)(is_left ? config_defs::joint_id::left : config_defs::joint_id::right) | (uint8_t)config_defs::joint_id::knee), exo_data)
, _ankle((config_defs::joint_id)((uint8_t)(is_left ? config_defs::joint_id::left : config_defs::joint_id::right) | (uint8_t)config_defs::joint_id::ankle), exo_data)
, _heel_fsr(is_left ? logic_micro_pins::fsr_sense_left_heel_pin : logic_micro_pins::fsr_sense_right_heel_pin) // Check if it is the left and use the appropriate pin for the side.
, _toe_fsr(is_left ? logic_micro_pins::fsr_sense_left_toe_pin : logic_micro_pins::fsr_sense_right_toe_pin)
{
    _data = exo_data;
    _is_left = is_left;
    // This data object is set for the specific leg so we don't have to keep checking the side.
    _leg_data = _is_left ? &(_data->left_leg) : &(_data->right_leg);
    
    _prev_heel_contact_state = true; // initialized to true so we don't get a strike the first time we read
    _prev_toe_contact_state = true;
    
    for (int i = 0; i<_num_steps_avg; i++)
    {
        _step_times[i] = 0;
    }
    _ground_strike_timestamp = 0;
    _prev_ground_strike_timestamp = 0;
    _expected_step_duration = 0;
};

/*
 * read FSR,  calc percent gait, read joint data, send joint commands
 */
void Leg::run_leg()
{
    check_calibration();
    
    // read all the data before we calculate and send the new motor commands
    read_data();
    
    // calculates the new motor commands and sends them.
    update_motor_cmds();
        
}; 

/*
 * Reads the FSR, detects ground strike, and calculates percent gait.
 * Sets the values to the corresponding place in data class.
 */
void Leg::read_data()
{
    // Check the FSRs
    _leg_data->heel_fsr = _heel_fsr.read();
    _leg_data->toe_fsr = _toe_fsr.read();
    _leg_data->ground_strike = _check_ground_strike();
    if (_leg_data->ground_strike)
    {
        _update_expected_duration();
    }
    _leg_data->percent_gait = _calc_percent_gait();
    
    // Check the joint sensors if the joint is used.
    if (_leg_data->hip.is_used)
    {
        _hip.read_data();
    }
    if (_leg_data->knee.is_used)
    {
        _knee.read_data();
    }
    if (_leg_data->ankle.is_used)
    {
        _ankle.read_data();
    }
};

/*
 * Checks if we need to do the calibration and runs the calibration if we do.
 */
void Leg::check_calibration()
{
    if (_leg_data->is_used)
    {
        // make sure fsr calibration is done before refinement.
        if (_leg_data->do_calibration_toe_fsr)
        {
            _leg_data->do_calibration_toe_fsr = _toe_fsr.calibrate(_leg_data->do_calibration_toe_fsr);
        }
        else if (_leg_data->do_calibration_refinement_toe_fsr) 
        {
            _leg_data->do_calibration_refinement_toe_fsr = _toe_fsr.refine_calibration(_leg_data->do_calibration_refinement_toe_fsr);
        }
        
        if (_leg_data->do_calibration_heel_fsr)
        {
            _leg_data->do_calibration_heel_fsr = _heel_fsr.calibrate(_leg_data->do_calibration_heel_fsr);
        }
        else if (_leg_data->do_calibration_refinement_heel_fsr) 
        {
            _leg_data->do_calibration_refinement_heel_fsr = _heel_fsr.refine_calibration(_leg_data->do_calibration_refinement_heel_fsr);
        }
        
        // check torque sensor calibrations if joint is use
        // Check the joint sensors if the joint is used.
        if (_leg_data->hip.is_used)
        {
            _hip.check_calibration();
        }
        if (_leg_data->knee.is_used)
        {
            _knee.check_calibration();
        }
        if (_leg_data->ankle.is_used)
        {
            _ankle.check_calibration();
        }
        
    }        
};

/*
 * Simple check for a rising edge of either FSR during swing and returns 1 if they have.
 */
bool Leg::_check_ground_strike()
{
    bool heel_contact_state = _heel_fsr.get_ground_contact();
    bool toe_contact_state = _toe_fsr.get_ground_contact();
    bool ground_strike = false;
    
    
    // Serial.print("Leg::_check_ground_strike : _prev_heel_contact_state - ");
    // Serial.println(_prev_heel_contact_state);
    // Serial.print("\t_prev_toe_contact_state - ");
    // Serial.println(_prev_toe_contact_state);
    // only check if in swing
    if(!_prev_heel_contact_state & !_prev_toe_contact_state)
    {
        //check for rising edge on heel and toe, toe is to account for flat foot landings
        if ((heel_contact_state > _prev_heel_contact_state) | (toe_contact_state > _prev_toe_contact_state))
        {
            ground_strike = true;
            _prev_ground_strike_timestamp = _ground_strike_timestamp;
            _ground_strike_timestamp = millis();
        }
    }
    _prev_heel_contact_state = heel_contact_state;
    _prev_toe_contact_state = toe_contact_state;
    
    return ground_strike;
};

/*
 * Uses the expected duration of the step to calculate the percent gait and returns the value
 * Saturates at 100%
 */
float Leg::_calc_percent_gait()
{
    int timestamp = millis();
    int percent_gait = -1;
    // only calulate if the expected step duration has been established.
    if (_expected_step_duration>0)
    {
        percent_gait = 100 * ((float)timestamp - _ground_strike_timestamp) / _expected_step_duration;
        percent_gait = min(percent_gait, 1000); // set saturation.
        // Serial.print("Leg::_calc_percent_gait : percent_gait_x10 = ");
        // Serial.println(percent_gait_x10);
    }
    return percent_gait;
};

/*
 * Calculates the expected duration of a step by averaging the time the last N steps took.
 * Should only be called when a ground strike has occurred.
 * 
 */
void Leg::_update_expected_duration()
{
    unsigned int step_time = _ground_strike_timestamp - _prev_ground_strike_timestamp;
		
    if (0 == _prev_ground_strike_timestamp) // if the prev time isn't set just return.
    {
        return;
    }
    uint8_t num_uninitialized = 0;
    // check that everything is set.
    for (int i = 0; i<_num_steps_avg; i++)
    {
        num_uninitialized += (_step_times[i] == 0);
    }
    
    // get the max and min values of the array for determining the window for expected values.
    unsigned int* max_val = std::max_element(_step_times, _step_times + _num_steps_avg);
    unsigned int* min_val = std::min_element(_step_times, _step_times + _num_steps_avg);
    
    if  (num_uninitialized > 0)  // if all the values haven't been replaced
    {
        // shift all the values and insert the new one
        for (int i = 1; i<_num_steps_avg; i++)
        {
            _step_times[i] = _step_times[i-1];
        }
        _step_times[0] = step_time;
        
        // Serial.print("Leg::_update_expected_duration : _step_times not fully initialized- [\t");
        // for (int i = 0; i<_num_steps_avg; i++)
        // {
            // Serial.print(_step_times[i]);
            // Serial.print("\t");
        // }
        // Serial.println("\t]");
        
        
    }
    // consider it a good step if the ground strike falls within a window around the expected duration.
    // Then shift the step times and put in the new value.
    else if ((step_time <= (_leg_data->expected_duration_window_upper_coeff * *max_val)) & (step_time >= (_leg_data->expected_duration_window_lower_coeff * *min_val))) // and (armed_time > ARMED_DURATION_PERCENT * self.expected_duration)): # a better check can be used.  If the person hasn't stopped or the step is good update the vector.  
    {
        int sum_step_times = step_time;
        for (int i = 1; i<_num_steps_avg; i++)
        {
            sum_step_times += _step_times[i-1];
            _step_times[i] = _step_times[i-1];
        }
        _step_times[0] = step_time;
        
        // TODO: Add rate limiter for change in expected duration so it can't make big jumps
        _expected_step_duration = sum_step_times/_num_steps_avg;  // Average to the nearest ms
        // Serial.print("Leg::_update_expected_duration : _expected_step_duration - ");
        // Serial.println(_expected_step_duration);
    }
};

/*
 * Reset the step times in case the value gets off and can't recover.
 */
void Leg::clear_step_time_estimate()
{
    for (int i = 0; i<_num_steps_avg; i++)
    {
        _step_times[i] = 0;
    }
};

/*
 * sends new command to the motors
 */
void Leg::update_motor_cmds()
{
    // Check the joint sensors if the joint is used.
    if (_leg_data->hip.is_used)
    {
        _hip.run_joint();
    }
    if (_leg_data->knee.is_used)
    {
        _knee.run_joint();
    }
    if (_leg_data->ankle.is_used)
    {
        _ankle.run_joint();
    }
};
#endif