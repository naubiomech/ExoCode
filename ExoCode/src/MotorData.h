/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef MotorData_h
#define MotorData_h

#include "Arduino.h"

#include "parseIni.h"
#include "board.h"

#include <stdint.h>

// forward declaration
class ExoData;

class MotorData 
{
	public:
        MotorData(config_defs::joint_id id, uint8_t* config_to_send);
        
        config_defs::joint_id id;
        uint8_t motor_type;
        float p; // read position
        float v; // read velocity
        float i; // read current
        float p_des;  
        float v_des;
        float kp = 0;
        float kd = 0;
        float t_ff;
        
        bool is_left;
};


#endif