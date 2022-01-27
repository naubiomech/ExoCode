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

#include <stdint.h>

class Leg
{
        
	public:
		//Leg();
        Leg(bool is_left, ExoData* exo_data); // constructor: 
		       
        void run_leg(); // read FSR,  calc percent gait, read joint data, send joint commands
		
		void read_data(); // reads motor data from each motor used in the leg and stores the values
		void update_motor_cmds();  // sends new control command to the motors used in the leg, based on the defined controllers
		void set_controller(int joint, int controller); // Changes the controller for an individual joint
		
	private:
		void _calc_percent_gait();
		ExoData* _data;
        
        Joint _hip;
        Joint _knee;
        Joint _ankle;
		FSR _heel_fsr;
		FSR _toe_fsr;
		
        bool _is_left;
		
};
#endif
#endif