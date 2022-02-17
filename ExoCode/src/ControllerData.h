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

namespace controller_defs
{
    namespace zero_torque
    {
        const uint8_t num_parameter = 0;
    }
    
    namespace proportional_joint_moment
    {
        const uint8_t max_torque_idx = 0;
        const uint8_t num_parameter = 1;
    }
    
    namespace heel_toe
    {
        const uint8_t max_torque_idx = 0;
        const uint8_t min_torque_idx = 1;
        const uint8_t num_parameter = 2;
    }
    
    const uint8_t max_parameters = 2;  // this should be the largest of all the num_parameters
}


class ControllerData {
	public:
        ControllerData(config_defs::joint_id id, uint8_t* config_to_send);
        
        uint8_t controller;
        config_defs::JointType joint; 
        int16_t setpoint;
        int16_t parameters[controller_defs::max_parameters];

        
};


#endif