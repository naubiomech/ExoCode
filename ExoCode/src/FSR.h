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
		
        /*
         * Does an initial time based calculation to find a rough range for the signal
         * Person should be walking
         * returns if the calibration is continuing.
         */
        bool calibrate(bool do_calibrate); // Changes the controller for an individual joint
		
        /*
         * Does a refinement of the calibration based on averaging across an number of steps.
         * Person should be walking.
         * returns if the calibration is continuing.
         */
        bool refine_calibration(bool do_refinement);
        
        /*
         * Reads the signal and applies the calibration.
         * returns the calibrated reading
         */
        float read(); // reads the pins and updates the data object.
		
        /*
         * returns if the FSR is in contact with the ground
         */
        bool get_ground_contact();
		
	private:
		/*
         * Calculates if the fsr is in contact with the ground based on a schmitt trigger
         * This is called in read()
         */
        bool _calc_ground_contact();  

        // Stores the sensor readings
        uint16_t _raw_reading;
		float _calibrated_reading;
        
        // The pin the sensor is connected to.
        int _pin;
        
        // used for calibration
        const uint16_t _cal_time = 5000; // this is time to do the initial calibration
        uint16_t _start_time;  // Stores the time we started the calibration
        bool _last_do_calibrate; //need to remember to delete this when the calibration ends.
        uint16_t _calibration_min;  // Minimum value during the time period
        uint16_t _calibration_max;  // Maximum value during the time period
        
        // used for calibration refinement
        const uint8_t _num_steps = 7; // this is the number of steps to do the calibration_refinement
        const float _lower_threshold_percent_calibration_refinement = .33;  // Lower threshold for the schmitt trigger. This can be relatively high since we don't really care about the exact moment the ground contact happens.
        const float _upper_threshold_percent_calibration_refinement = .66;  // Upper threshold for the schmitt trigger
        bool _state;  // Stores the signal high/low state from the schmitt trigger to find when there is a new step.
        bool _last_do_refinement;  // Used to track the rising edge of do_refinement, so we can reset on the first run.
        unsigned int _step_max_sum; // stores the running sum of maximums from each step so we can average.
        uint16_t _step_max; // keeps track of the max value for the step.
        unsigned int _step_min_sum; // same as max
        uint16_t _step_min; // same as max
        uint8_t _step_count;  // Used to track if we have done the required number of steps.
        float _calibration_refinement_min;  // The refined min used for doing the calibration
        float _calibration_refinement_max;  // The refined max used for doing the calibration
        
        // used for ground_contact()
        bool _ground_contact;  // is the FSR in contact with the ground
        const float _lower_threshold_percent_ground_contact = .15;  // Lower threshold for the schmitt trigger.  This should be relatively low as we want to detect as close to ground contact as possible.
        const float _upper_threshold_percent_ground_contact = .25;  // Should be slightly higher than the lower threshold but by as little as you can get by with as the sensor must go above this value to register contact. 
        
};
#endif
#endif