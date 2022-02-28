/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef TorqueSensor_h
#define TorqueSensor_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)

#include "Board.h"
#include "Arduino.h"

class TorqueSensor
{
	public:
		TorqueSensor(unsigned int pin);
        
        /*
         * Does the calibration, while do_calibration is true, and returns 0 when the calibration has finished.
         */
        bool calibrate(bool do_calibration); 
		
        /*
         * reads the pins and updates the data object.
         */
        int read();
				
		
	private:
		bool _is_used;
        int _pin;
        int _calibration;  // Stores the value used for calibration.
        int _raw_reading;
		int _calibrated_reading;
        
        const uint16_t _cal_time = 1000; // this is time to do the initial calibration
        uint16_t _start_time;  // time the calibration starts.
        bool _last_do_calibrate; //need to remember to delete this when the calibration ends.
        int _zero_sum; // sum of values over the calibration period used for averaging.
        uint16_t _num_calibration_samples;  // number of samples collected during calibration, denominator for averaging.
        
};
#endif
#endif