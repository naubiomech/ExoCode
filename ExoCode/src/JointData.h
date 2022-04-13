/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef JointData_h
#define JointData_h


#include "Arduino.h"

#include "MotorData.h"
#include "ControllerData.h"
#include "ParseIni.h"
#include "Board.h"


#include <stdint.h>


// forward declaration
class ExoData;

class JointData {
	public:
        JointData(config_defs::joint_id id, uint8_t* config_to_send);
        
        config_defs::joint_id id;
        MotorData motor;
        ControllerData controller;
        int torque_reading;  // reading from the torque sensor
        bool is_left;
        bool flip_direction;
        bool is_used;
        bool calibrate_torque_sensor;  // flag for if we should calibrate.
        
        float position;
        float velocity;
};


#endif