/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Motor_h
#define Motor_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)

#include "Arduino.h"

#include "ExoData.h"
#include "parseIni.h"
#include "board.h"

#include <stdint.h>

class Motor
{
	public:
		Motor(config_defs::joint_id id, ExoData* exo_data); // constructor: type is the motor type
		
		void read_data(); // reads motor data from each motor used in the leg and stores the values
		void send_data();  // sends new control command to the motors used in the leg, based on the defined controllers
		void set_controller(int controller); // Changes the low level controller for an individual joint
		void motor_on_off(bool on);  // motor enable/disable
		bool get_is_left();  // lets you know if it is a left or right leg.
			
		config_defs::joint_id id; //motor id
		// float p; // read position
		// float v; // read velocity
		// float i; // read current
		
		// float p_des; // 
		// float v_des;
		// float kp;
		// float kd;
		// float t_ff;
		
		
	private:
		ExoData* _data;
        
        // int _p;
		// int _v;
		// int _i;
		bool _is_left;
};
#endif
#endif