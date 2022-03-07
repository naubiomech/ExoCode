/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef LegData_h
#define LegData_h

#include "Arduino.h"

#include "JointData.h"
#include "ParseIni.h"
#include "Board.h"

#include <stdint.h>

// forward declaration
class ExoData;


class LegData{
	   
    public:
        LegData(bool is_left, uint8_t* config_to_send);
        
        JointData hip;
        JointData knee;
        JointData ankle; 
        
        
        float percent_gait; // likely want to do fixed point 
        // Calibrated FSR readings
        float heel_fsr;
        float toe_fsr;
        
        // Trigger when we go from swing to one FSR making contact.
        bool ground_strike;
        
        bool is_left;
        bool is_used;
        bool do_calibration_toe_fsr; //bit 0 is calibrate fsr, bit 1 is refine calibration.
        bool do_calibration_refinement_toe_fsr; 
        bool do_calibration_heel_fsr; //bit 0 is calibrate fsr, bit 1 is refine calibration.
        bool do_calibration_refinement_heel_fsr; 
        
        // Sets the window to determine if a ground strike is considered a new step.
        float expected_duration_window_upper_coeff;
        float expected_duration_window_lower_coeff;
};

#endif