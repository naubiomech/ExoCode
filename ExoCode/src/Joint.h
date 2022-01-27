/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Joint_h
#define Joint_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)

#include "Arduino.h"

#include "ExoData.h"
#include "Motor.h"
#include "Controller.h"
#include "TorqueSensor.h"
#include "parseIni.h"
#include "board.h"
#include "Joint.h"

#include <stdint.h>

class Joint
{
	static uint8_t left_used_joint_count;
    static uint8_t right_used_joint_count;
    
    public:
		Joint(config_defs::joint_id id, ExoData* exo_data);  // constructor:  
		void run_joint();  // updates the controller and sends the motor command
		void read_data(); // reads data from motor and sensors
		void set_controller(uint8_t);  // changes the high level controller in Controller, and the low level controller in Motor
		
        // create some static member functions we can use for the initializer list.
        static bool get_is_left(config_defs::joint_id);
        static uint8_t get_joint_type(config_defs::joint_id);
        static unsigned int get_torque_sensor_pin(config_defs::joint_id, ExoData*);
        
		config_defs::joint_id id;
        bool is_left;
        
        
	private:
		ExoData* _data;
        
        
        Motor _motor;
		TorqueSensor _torque_sensor;
		Controller _controller;
        
        
};

#endif
#endif