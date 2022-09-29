#include "TorqueSensor.h"
//#define TORQUE_DEBUG 1

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 
/*
 * Constructor for the TorqueSensor
 * Takes in the pin to use and sets it as an analog input if it is not the NC pin.
 * NC pin is used if the joint the object is associated with is not used, or if all the available sensor slots have been taken.
 * Calibration and readings are initialized to 0 
 */
TorqueSensor::TorqueSensor(unsigned int pin)
{
    this->_is_used = (pin == logic_micro_pins::not_connected_pin?  false:true);
    this->_pin = pin;
    this->_calibration = 0;
    this->_raw_reading = 0;
    this->_calibrated_reading = 0;
    this->_start_time = 0;
    this->_last_do_calibrate = false; 
    this->_zero_sum = 0; 
    this->_num_calibration_samples = 0;  
        
    // Serial.print("TorqueSensor::TorqueSensor : pin = ");
    // Serial.print(pin);
    // Serial.print("\n");
    
    // Configure pin if it is used
    if (this->_is_used)
    {
        pinMode(this->_pin, INPUT);  // Not required but I like to be explicit
    }
    
    #ifdef TORQUE_DEBUG
        Serial.print("TorqueSensor :: Constructor : pin ");
        Serial.print(_pin);
        Serial.println(" set");
    #endif
};

bool TorqueSensor::calibrate(bool do_calibrate)
{
    #ifdef TORQUE_DEBUG
        Serial.print("TorqueSensor::calibrate : do_calibrate = ");
        Serial.print(do_calibrate);
        Serial.print("\n");
    #endif
    if (_is_used)
    {
        #ifdef TORQUE_DEBUG
        Serial.print("TorqueSensor::calibrate : _is_used\n");
        #endif
        // check for rising edge of do_calibrate, and reset the values
        if (do_calibrate > _last_do_calibrate)
        {
            _start_time = millis();
            _zero_sum = 0;
            _num_calibration_samples = 0;
        }
        // check if we are within the time window and need to do the calibration
        if((_cal_time >= (millis()-_start_time)) & do_calibrate)
        {
            uint16_t current_reading = analogRead(_pin);
            _zero_sum = _zero_sum + current_reading;
            _num_calibration_samples++;
            
        }
        // The time window ran out so we are done, and should average the values and set the _calibration value.
        else if (do_calibrate)
        {
            if (_num_calibration_samples>0)
            {
                _calibration = _zero_sum/_num_calibration_samples;
            }
            #ifdef TORQUE_DEBUG
            Serial.print("TorqueSensor::calibrate : Torque Cal Done\n");
            #endif
            do_calibrate = false;
        }
        
        // find the min and max over the time interval
        
        // offset by min and normalize by (max-min), (val-avg_min)/(avg_max-avg_min)
        
        // check if we are done with the calibration 
        _last_do_calibrate = do_calibrate;
    }
    else
    {
        #ifdef TORQUE_DEBUG
        Serial.print("TorqueSensor::calibrate : Not _is_used\n");
        #endif
        do_calibrate = false;
    }
    return do_calibrate;
};

float TorqueSensor::read()
{
    _raw_reading = analogRead(_pin);
    _calibrated_reading = ((float)_raw_reading - _calibration) * torque_calibration::AI_CNT_TO_V * torque_calibration::TRQ_V_TO_NM;

    #ifdef TORQUE_DEBUG
        Serial.print("TorqueSensor :: Read : pin ");
        Serial.print(_pin);
        Serial.print(" : Calibrated Reading = ");
        Serial.println(_calibrated_reading, 12);

        Serial.print("TorqueSensor :: Read : pin ");
        Serial.print(_pin);
        Serial.print(" : Raw Reading = ");
        Serial.println(_raw_reading);
        Serial.println(" ");

        Serial.print("_calibration : ");
        Serial.println(_calibration);
        Serial.print("torque_calibration::AI_CNT_TO_V : ");
        Serial.println(torque_calibration::AI_CNT_TO_V, 12);
        Serial.print("torque_calibration::TRQ_V_TO_NM : ");
        Serial.println(torque_calibration::TRQ_V_TO_NM);
    #endif
    
    return _calibrated_reading;
};


#endif