/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Leg_h
#define Leg_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)

#include "Arduino.h"

#include "joint.h"
#include "controller.h"
#include "FSR.h"
#include "parseIni.h"
#include "board.h"
#include "Utilities.h"

#include <stdint.h>
#include <algorithm>

class Leg
{
    public:
		//Leg();
        Leg(bool is_left, ExoData* exo_data); // constructor: 
        
        /*
         * read FSR,  calc percent gait, read joint data, send joint commands
         */
        void run_leg(); 
		
        /*
         * Checks if calibration flags are set, and runs calibration if they are.
         */
        void check_calibration();  
		
        /*
         * Reads motor data from each motor used in the leg and stores the values
         */
        void read_data(); 
		
        /*
         * sends new control command to the motors used in the leg, based on the defined controllers
         */
        void update_motor_cmds();   
		
        /*
         * Changes the controller for an individual joint
         */
        void set_controller(int joint, int controller);  
		
        /*
         * Simply clears the step time estimate for when it gets off by more than can be adjusted for.
         */
        void clear_step_time_estimate();
        
        // TEMP move to test FSRs and control, move back to private when done
        FSR _heel_fsr;
		FSR _toe_fsr;
        AnkleJoint _ankle;
        HipJoint _hip;
	private:
		
        /*
         * Calculates the percent of gait based on the ground contact reading
         * and an estimate of the step time based on the average time of the last few steps.
         * returns the percent gait * 10
         */
        int _calc_percent_gait();
        
        /*
         * Creates an average step time based on the last few steps
         */
        void _update_expected_duration();
        
        /*
         * Checks for state changes in the FSRs to find the point when ground contact is made
         * The returned value should just be high for a single cycle.
         */
        bool _check_ground_strike();
		
        // data that can be accessed
        ExoData* _data;
        LegData* _leg_data;// breaks out the specific leg we are using so we don't have to keep checking if it is left.
        
        // joint objects for the leg.
        
        KneeJoint _knee;
        
		// stores which side the leg is on.
        bool _is_left;
        
        // used for ground strike detection
        bool _prev_heel_contact_state;
        bool _prev_toe_contact_state;
        
        // used for percent gait calculation
        static const uint8_t _num_steps_avg = 3; // the number of prior steps used to estimate the expected duration
        unsigned int _step_times[_num_steps_avg]; // stores the duration of the last N steps
        
        unsigned int _ground_strike_timestamp;  // Records the time of the ground strike to determine if the next strike is within the expected window.
        unsigned int _prev_ground_strike_timestamp;  // Stores the last value to determine the difference in strike times.
        unsigned int _expected_step_duration;  // The expected step duration to calculate the percent gait.
        
};
#endif
#endif