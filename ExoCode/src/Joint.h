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
#include "Utilities.h"

#include <stdint.h>

class _Joint
{
	static uint8_t left_used_joint_count;
    static uint8_t right_used_joint_count;
    
    
    // TODO: create object for each type of controller (joint type specific) and pointer to current controller.
    // TODO: Create object for specific motor used based on config.
    // TODO: Create joint base class that can then be used for each joint so that hip, knee, and ankle can each have joint specific controllers.
    public:
		_Joint(config_defs::joint_id id, ExoData* exo_data);  // constructor:  
		virtual ~_Joint(){};
        
        virtual void run_joint() = 0;  // updates the controller and sends the motor command
		virtual void read_data() = 0; // reads data from motor and sensors
		virtual void set_controller(uint8_t) = 0;  // changes the high level controller in Controller, and the low level controller in Motor
		
        // create some static member functions we can use for the initializer list.
        static bool get_is_left(config_defs::joint_id);
        static uint8_t get_joint_type(config_defs::joint_id);
        static unsigned int get_torque_sensor_pin(config_defs::joint_id, ExoData*);
        
        _Motor* _motor;
		TorqueSensor _torque_sensor;
		_Controller* _controller;
	
    private:
		ExoData* _data;
        config_defs::joint_id _id;
        bool _is_left;
        
        
};

class HipJoint : public _Joint
{
    public:
        HipJoint(config_defs::joint_id id, ExoData* exo_data);
        ~HipJoint(){};
        
        void run_joint();  // updates the controller and sends the motor command
        void read_data(); // reads data from motor and sensors
        void set_controller(uint8_t);  // changes the high level controller in Controller, and the low level controller in Motor
    
    
};

class KneeJoint : public _Joint
{
    public:
        KneeJoint(config_defs::joint_id id, ExoData* exo_data);
        ~KneeJoint(){};
        
        void run_joint();  // updates the controller and sends the motor command
        void read_data(); // reads data from motor and sensors
        void set_controller(uint8_t);  // changes the high level controller in Controller, and the low level controller in Motor
		
};

class AnkleJoint : public _Joint
{
    public:
        AnkleJoint(config_defs::joint_id id, ExoData* exo_data);
        ~AnkleJoint(){};
        
        void run_joint();  // updates the controller and sends the motor command
        void read_data(); // reads data from motor and sensors
        void set_controller(uint8_t);  // changes the high level controller in Controller, and the low level controller in Motor
		
};

#endif
#endif