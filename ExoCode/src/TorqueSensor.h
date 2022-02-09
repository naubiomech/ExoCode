/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef TorqueSensor_h
#define TorqueSensor_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)

#include "board.h"
#include "Arduino.h"

class TorqueSensor
{
	public:
		TorqueSensor(unsigned int pin);
        bool calibrate(bool do_calibration); // Changes the controller for an individual joint
		int read(); // reads the pins and updatas the data stuct.
				
		
	private:
		bool _is_used;
        int _pin;
        int _calibration;
        int _raw_reading;
		int _calibrated_reading;
        
        const uint16_t _cal_time = 1000; // this is time to do the initial calibration
        uint16_t _start_time;
        bool _last_do_calibrate; //need to remember to delete this when the calibration ends.
        int _zero_sum;
        uint16_t _num_calibration_samples;
        
};
#endif
#endif