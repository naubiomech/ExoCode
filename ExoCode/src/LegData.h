/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef LegData_h
#define LegData_h

#include "Arduino.h"

#include "JointData.h"
#include "parseIni.h"
#include "board.h"

#include <stdint.h>

// forward declaration
class ExoData;

class LegData{
	   
    public:
        LegData(bool is_left, uint8_t* config_to_send);
        
        JointData hip;
        JointData knee;
        JointData ankle; 
        
        
        int percent_gait; // likely want to do fixed point 
        int heel_fsr;
        int toe_fsr;
        bool is_left;
        bool is_used;
        bool calibrate_fsr;
        
};

#endif