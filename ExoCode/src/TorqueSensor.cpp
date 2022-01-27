/*
 * 
 * P. Stegall Jan. 2022
*/

#include "TorqueSensor.h"

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36) 
/*
 * Constructor for the TorqueSensor
 * Takes in the pin to use and sets it as an analog input if it is not the NC pin.
 * NC pin is used if the joint the object is associated with is not used, or if all the available sensor slots have been taken.
 * Calibration and readings are initialized to 0 
 */
TorqueSensor::TorqueSensor(unsigned int pin)
{
    this->_is_used = (pin == logic_micro_pins::not_connected_pin?  true:false);
    this->_pin = pin;
    this->_calibration = 0;
    this->_rawReading = 0;
    this->_calibratedReading = 0;
    
    // Configure pin if it is used
    if (this->_is_used)
    {
        
    }
};
#endif