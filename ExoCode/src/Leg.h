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
		       
        void run_leg(); // read FSR,  calc percent gait, read joint data, send joint commands
		
        void check_calibration();  //Checks if calibration flags are set, and runs calibration if they are.
		void read_data(); // reads motor data from each motor used in the leg and stores the values
		void update_motor_cmds();  // sends new control command to the motors used in the leg, based on the defined controllers
		void set_controller(int joint, int controller); // Changes the controller for an individual joint
		void clear_step_time_estimate();
        // temp move to test FSRs, move back to private when done
        FSR _heel_fsr;
		FSR _toe_fsr;
        AnkleJoint _ankle;
	private:
		int _calc_percent_gait();
        void _update_expected_duration();
        
        bool _check_ground_strike();
		
        ExoData* _data;
        LegData* _leg_data;// breaks out the specific leg we are using so we don't have to keep checking if it is left.
        
        HipJoint _hip;
        KneeJoint _knee;
        
				
        bool _is_left;
        
        // used for ground strike detection
        bool _prev_heel_contact_state;
        bool _prev_toe_contact_state;
        
        // used for percent gait calculation
        static const uint8_t _num_steps_avg = 3; //sizeof _step_times / sizeof _step_times[0];
        unsigned int _step_times[_num_steps_avg] ;
        
        unsigned int _ground_strike_timestamp;
        unsigned int _prev_ground_strike_timestamp;
        unsigned int _expected_step_duration;
        
};
#endif
#endif