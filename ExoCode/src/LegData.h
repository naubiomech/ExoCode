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
        
        
        int percent_gait_x10; // likely want to do fixed point 
        float heel_fsr;
        float toe_fsr;
        
        bool ground_strike;
        
        bool is_left;
        bool is_used;
        bool do_calibration_toe_fsr; //bit 0 is calibrate fsr, bit 1 is refine calibration.
        bool do_calibration_refinement_toe_fsr; 
        bool do_calibration_heel_fsr; //bit 0 is calibrate fsr, bit 1 is refine calibration.
        bool do_calibration_refinement_heel_fsr; 
};

#endif