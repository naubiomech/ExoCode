/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef MotorData_h
#define MotorData_h

#include "Arduino.h"

#include "ParseIni.h"
#include "Board.h"

#include <stdint.h>

// forward declaration
class ExoData;

class MotorData 
{
	public:
        MotorData(config_defs::joint_id id, uint8_t* config_to_send);
        void reconfigure(uint8_t* config_to_send);
        
        config_defs::joint_id id;
        uint8_t motor_type;
        float p; // read position
        float v; // read velocity
        float i; // read current
        float p_des = 0;  
        float v_des = 0;
        float kp = 0;
        float kd = 0;
        float t_ff = 0;
        
        // add do_zero_flag and check in run.
        bool do_zero;
        bool enabled;
        bool is_on;
        bool is_left;
        bool flip_direction;
        float gearing;
};


#endif