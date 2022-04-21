/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Joint_h
#define Joint_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "Arduino.h"

#include "ExoData.h"
#include "Motor.h"
#include "Controller.h"
#include "TorqueSensor.h"
#include "ParseIni.h"
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
        
        /*
         * updates the controller and sends the motor command
         */
        virtual void run_joint() = 0;   
		
        /*
         * reads data from motor and sensors
         */
        virtual void read_data(); 

        /*
         * Checks if we need to do the calibration for the motor and sensors
         * and runs the calibration.
         */
        virtual void check_calibration();         
		
        /*
         * changes the high level controller in Controller, and the low level controller in Motor
         */
        virtual void set_controller(uint8_t) = 0;  
		
        /*
         * Sets the motor to use.  Not strictly needed since everything stays internal.
         */
        void set_motor(_Motor* new_motor);
        
        // create some static member functions we can use for the initializer list.
        
        /*
         * Takes in the joint id and exo data, and checks if the current joint is used.
         * If it is used it pulls the next open torque sensor pin for the side, and increments the counter.
         * If the joint is not used, or we have used up all the available torque sensor pins for the side, it sets the pin to a pin that is not connected.
         */
        static unsigned int get_torque_sensor_pin(config_defs::joint_id, ExoData*);
        
        //TODO: Make these protected
        _Motor* _motor; // using pointer to the base class so we can use any motor type.
        JointData* _joint_data;
    protected:
        // give access to the larger data object and the joint specific data 
        ExoData* _data;
        
        
        // IO objects for the joint
        //_Motor* _motor; // using pointer to the base class so we can use any motor type.
		TorqueSensor _torque_sensor;
		_Controller* _controller; // Using pointer so we just need to change the object we are pointing to when the controller changes.
        
        // joint info
        config_defs::joint_id _id;
        bool _is_left;
    
};

class HipJoint : public _Joint
{
    public:
        HipJoint(config_defs::joint_id id, ExoData* exo_data);
        ~HipJoint(){};
        
        void run_joint();  // See _Joint 
        //void read_data(); // See _Joint 
        void set_controller(uint8_t);  // See _Joint 
    protected:
        // Objects for joint specific controllers
        ZeroTorque _zero_torque;
        HeelToe _heel_toe;
        ExtensionAngle _extension_angle;
        FranksCollinsHip _franks_collins_hip;
};

class KneeJoint : public _Joint
{
    public:
        KneeJoint(config_defs::joint_id id, ExoData* exo_data);
        ~KneeJoint(){};
        
        void run_joint();  // See _Joint
        //void read_data(); // See _Joint
        void set_controller(uint8_t);  // See _Joint
	
    protected:
        // Objects for joint specific controllers	
        ZeroTorque _zero_torque;
};

class AnkleJoint : public _Joint
{
    public:
        AnkleJoint(config_defs::joint_id id, ExoData* exo_data);
        ~AnkleJoint(){};
        
        void run_joint();  // See _Joint
        //void read_data(); // See _Joint
        void set_controller(uint8_t);  // See _Joint
		
    protected:
        // Objects for joint specific controllers
        ZeroTorque _zero_torque;
        ProportionalJointMoment _proportional_joint_moment;
        ZhangCollins _zhang_collins;
};

#endif
#endif