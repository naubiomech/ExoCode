/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Controllerdata_h
#define ControllerData_h
#include <stdint.h>

#include "Arduino.h"

#include "board.h"
#include "parseIni.h"
#include <stdint.h>

// forward declaration
class ExoData;


class ControllerData {
	public:
        ControllerData(config_defs::joint_id id, uint8_t* config_to_send);
        
        uint8_t controller;
        config_defs::JointType joint; 
        int setpoint;
        int parameter1;
        int parameter2;
        
};


#endif