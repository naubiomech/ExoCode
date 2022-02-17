/*
 * 
 * P. Stegall Jan. 2022
*/

#include "FSR.h"

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36) 
/*
 * Constructor for the force sensitive resistor
 * Takes in the pin to use and sets it as an analog input
 * Calibration and readings are initialized to 0 
 */
FSR::FSR(int pin)
{
    _pin = pin;
    
    
    _raw_reading = 0;
    _calibrated_reading = 0;
    
    
    _last_do_calibrate = false; //need to remember to delete this when the calibration ends.
    _start_time = 0;
    _calibration_min = 0;
    _calibration_max = 0;
    
    _state = false;
    _last_do_refinement = false;
    _step_count = 0;
    _calibration_refinement_min = 0;
    _calibration_refinement_max = 0;
}

/*
 * This gets a rough estimate of the calibrated values.
 * These values will be used for tracking the number of transitions for the calibration refinement. 
 *
 */
bool FSR::calibrate(bool do_calibrate)
{
    // check for rising edge of do_calibrate
    if (do_calibrate > _last_do_calibrate)
    {
        _start_time = millis();
    }
    
    if((_cal_time >= (millis()-_start_time)) & do_calibrate)
    {
        uint16_t current_reading = analogRead(_pin);
        _calibration_max = max(_calibration_max, current_reading);
        _calibration_min = min(_calibration_min, current_reading);
    }        
    else if (do_calibrate)
    {
        // Serial.println("FSR::calibrate : FSR Cal Done");
        // Serial.print("FSR::calibrate : _calibration_max - ");
        // Serial.println(_calibration_max);
        do_calibrate = false;
    }
    
    // find the min and max over the time interval
    
    // offset by min and normalize by (max-min), (val-avg_min)/(avg_max-avg_min)
    
    // check if we are done with the calibration 
    _last_do_calibrate = do_calibrate;
    return do_calibrate;
};

/*
 * Refines the calibration, by finding the max and min over a set number of low to high transitions
 */
bool FSR::refine_calibration(bool do_refinement)
{
    if (do_refinement)
    {
        // check for rising edge of do_calibrate
        if (do_refinement > _last_do_refinement)
        {
            _step_count = 0;
            
            // set the step max min to the middle value so the initial value is likely not used.
            _step_max = (_calibration_max+_calibration_min)/2;
            _step_min = (_calibration_max+_calibration_min)/2;
            
            // reset the sum that will be used for averaging
            _step_max_sum = 0;
            _step_min_sum = 0;
        }
        
        // check if we are done with the calibration
        if (_step_count < _num_steps)
        {
            uint16_t current_reading = analogRead(_pin);
            
            // For each step find max and min for every step
            // keep a running record of the max and min for the step.
            _step_max = max(_step_max, current_reading);
            _step_min = min(_step_min, current_reading);
            
            
            bool last_state = _state;
            _state = utils::schmitt_trigger(current_reading, last_state, _lower_threshold_percent_calibration_refinement * (_calibration_max-_calibration_min) + _calibration_min, _upper_threshold_percent_calibration_refinement * (_calibration_max-_calibration_min) + _calibration_min); 
            
            // there is a new low -> high transition (next step), add the step max and min to their respective sums.
            if (_state > last_state) 
            {
                _step_max_sum = _step_max_sum + _step_max;
                _step_min_sum = _step_min_sum + _step_min;
                
                // reset the step max/min tracker for the next step
                _step_max = (_calibration_max+_calibration_min)/2;
                _step_min = (_calibration_max+_calibration_min)/2;
                
                _step_count++;
                // Serial.print("FSR::refine_calibration : New Step - ");
                // Serial.println(_step_count);
            }
        }
        else // we are still at do_refinement but the _step_count is at the _num_steps
        {
            // set the calibration as the average of the max values
            // average max and min, offset by min and normalize by (max-min), (val-avg_min)/(avg_max-avg_min)
            _calibration_refinement_max = static_cast<decltype(_calibration_refinement_max)>(_step_max_sum)/_num_steps;  // casting to the type of _calibration_refinement_max before division 
            _calibration_refinement_min = static_cast<decltype(_calibration_refinement_min)>(_step_min_sum)/_num_steps;
            
            // Serial.println("FSR::refine_calibration : FSR Cal Done");
            // Serial.print("FSR::refine_calibration : New Step - ");
            // Serial.println(_step_count);
            
            // Serial.print("FSR::refine_calibration : _calibration_refinement_max - ");
            // Serial.println(_calibration_refinement_max);
            // Serial.print("FSR::refine_calibration : _calibration_refinement_min - ");
            // Serial.println(_calibration_refinement_min);
            //delay(1000);
            
            // refinement is done
            do_refinement = false;
        } 
    }
    _last_do_refinement = do_refinement;
    return do_refinement;
};

float FSR::read()
{
    _raw_reading = analogRead(_pin);
    if (_calibration_refinement_max > 0)
    {
        _calibrated_reading = ((float)_raw_reading - _calibration_refinement_min)/(_calibration_refinement_max-_calibration_refinement_min);
    }
    else if (_calibration_max > 0)
    {
        _calibrated_reading = ((float)_raw_reading - _calibration_min)/(_calibration_max-_calibration_min);
    }
    else
    {
        _calibrated_reading = _raw_reading;
    }
    
    _calc_ground_contact();
    return _calibrated_reading;

};


bool FSR::_calc_ground_contact()
{
    if (_calibration_refinement_max > 0)
    {
        _ground_contact = utils::schmitt_trigger(_calibrated_reading, _ground_contact, _lower_threshold_percent_ground_contact, _upper_threshold_percent_ground_contact); 
    }
    return _ground_contact;
};

bool FSR::get_ground_contact()
{
    return _ground_contact;
};
#endif