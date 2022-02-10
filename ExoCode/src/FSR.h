/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef FSR_h
#define FSR_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)

#include "Arduino.h"
#include "board.h"
#include "Utilities.h"


class FSR
{
	public:
		FSR(int pin);
		bool calibrate(bool do_calibrate); // Changes the controller for an individual joint
		bool refine_calibration(bool do_refinement);
        float read(); // reads the pins and updatas the data stuct.
		bool edge_detector(bool check_rising);  // finds the rising or falling edge of the signal.
			
		
	private:
		uint16_t _raw_reading;
		float _calibrated_reading;
        
        int _pin;
        
        const uint16_t _cal_time = 5000; // this is time to do the initial calibration
        uint16_t _start_time;
        bool _last_do_calibrate; //need to remember to delete this when the calibration ends.
        uint16_t _calibration_min;
        uint16_t _calibration_max;
        
        const uint8_t _num_steps = 7; // this is the number of steps to do the calibration_refinement
        const float _lower_threshold_percent = .33;
        const float _upper_threshold_percent = .66;
        bool _state;
        bool _last_do_refinement;
        unsigned int _step_max_sum;
        uint16_t _step_max;
        unsigned int _step_min_sum;
        uint16_t _step_min;
        uint8_t _step_count;
        float _calibration_refinement_min;
        float _calibration_refinement_max;
        
};
#endif
#endif