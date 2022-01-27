/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Controller_h
#define Controller_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)

#include "Arduino.h"

#include "ExoData.h"
#include "board.h"
#include "parseIni.h"
#include <stdint.h>

class Controller
{
	public:
		Controller(config_defs::joint_id id, ExoData* exo_data);
		void set_controller(int joint, int controller); // Changes the controller for an individual joint
		int calc_motor_cmd();
        void update_setpoint(int setpoint);
        void update_parameters(int parameter1, int parameter2);
		
        config_defs::joint_id id;
		
		
	private:
        ExoData* _data;
        
        // int _setpoint;
		// int _parameter1;
		// int _parameter2;
        
        
};
#endif
#endif