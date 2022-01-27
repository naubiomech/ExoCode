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
    this->_pin = pin;
    
    
    this->_calibration = 0;
    this->_rawReading = 0;
    this->_calibratedReading = 0;
}
#endif