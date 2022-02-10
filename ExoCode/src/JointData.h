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
#include "parseIni.h"
#include "board.h"


#include <stdint.h>


// forward declaration
class ExoData;

class JointData {
	public:
        JointData(config_defs::joint_id id, uint8_t* config_to_send);
        
        config_defs::joint_id id;
        MotorData motor;
        ControllerData controller;
        int torque_reading;
        bool is_left;
        bool is_used;
        bool calibrate_torque_sensor;
};


#endif