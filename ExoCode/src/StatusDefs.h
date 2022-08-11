/*
 * Class to set an RGB LED to different colors based on the state of the system
 * 
 * Constructor: StatusLed(int r_pin, int g_pin, int b_pin) or (int r_pin, int g_pin, int b_pin, int brightness)
 *   The pins are the RGB LED pins ideally they are PWM but can also handle simple digital pins.
 *      In the header set NO_PWM to true or false depending on if you have PWM or simple digital pins.
 *   Brightness sets the brightness from 255 to 0, this is ignored for simple digital pins
 *   
 * updateLed(int message) method sets the color of the LED, these messages can be found in the preprocessor part of the header.
 * setBrightness(int brightness) method is used to change the brightness after initialization.
 * 
 * P. Stegall Dec. 2021
*/


#ifndef StatusDefs_h
#define StatusDefs_h

#include "Arduino.h"
#include "Board.h"

namespace status_defs
{
    namespace messages 
    {
        
        //0b error ,trial_on, trial_off
        const uint16_t off =  0;
        const uint16_t trial_off = 1;
        const uint16_t trial_on = 2;
        const uint16_t test = 3; // generally won't be used.
        const uint16_t torque_calibration = 4;
        const uint16_t fsr_calibration = 5;
        const uint16_t fsr_refinement =6;
        // Specific error messages will use the 4 highest bits, giving 31 error messages
        const uint16_t error_bit = 4;  // gives 12 bits of error messages
        const uint16_t error = 1<<(error_bit-1);
        
        const uint16_t error_left_heel_fsr =  1<<error_bit | error;
        const uint16_t error_left_toe =  2<<error_bit | error;
        const uint16_t error_right_heel_fsr =  3<<error_bit | error;
        const uint16_t error_right_toe_fsr =  4<<error_bit | error;
        
        const uint16_t error_left_hip_torque_sensor =  5<<error_bit | error;
        const uint16_t error_left_knee_torque_sensor =  6<<error_bit | error;
        const uint16_t error_left_ankle_torque_sensor =  7<<error_bit | error;
        const uint16_t error_right_hip_torque_sensor =  8<<error_bit | error;
        const uint16_t error_right_knee_torque_sensor =  9<<error_bit | error;
        const uint16_t error_right_ankle_torque_sensor =  10<<error_bit | error;
        
        const uint16_t error_left_hip_motor =  11<<error_bit | error;
        const uint16_t error_left_knee_motor =  12<<error_bit | error;
        const uint16_t error_left_ankle_motor =  13<<error_bit | error;
        const uint16_t error_right_hip_motor =  14<<error_bit | error;
        const uint16_t error_right_knee_motor =  15<<error_bit | error;
        const uint16_t error_right_ankle_motor =  16<<error_bit | error;
        
        const uint16_t error_left_hip_controller =  17<<error_bit | error;
        const uint16_t error_left_knee_controller =  18<<error_bit | error;
        const uint16_t error_left_ankle_controller =  19<<error_bit | error;
        const uint16_t error_right_hip_controller =  20<<error_bit | error;
        const uint16_t error_right_knee_controller =  21<<error_bit | error;
        const uint16_t error_right_ankle_controller =  22<<error_bit | error; 
        
        const uint16_t error_to_be_used_1 =  23<<error_bit | error;
        const uint16_t error_to_be_used_2 =  24<<error_bit | error;
        const uint16_t error_to_be_used_3 =  25<<error_bit | error;
        const uint16_t error_to_be_used_4 =  26<<error_bit | error;
        const uint16_t error_to_be_used_5 =  27<<error_bit | error;
        const uint16_t error_to_be_used_6 =  28<<error_bit | error;
        const uint16_t error_to_be_used_7 =  29<<error_bit | error;
        const uint16_t error_to_be_used_8 =  30<<error_bit | error;
        const uint16_t error_to_be_used_9 =  31<<error_bit | error;
        const uint16_t error_to_be_used_10 =  32<<error_bit | error;
        const uint16_t error_to_be_used_11 =  33<<error_bit | error;
        const uint16_t error_to_be_used_12 =  34<<error_bit | error; 
        const uint16_t error_to_be_used_13 =  35<<error_bit | error;
        
        const uint16_t warning_bit = error_bit + 6 + 1; //error bit plus 2^6-1 error messages, bit 11 for warning and 2^5-1 warning messages.
        const uint16_t warning = 1<<(warning_bit-1);
        const uint16_t warning_exo_run_time =  1<<warning_bit | warning;  
        
        const uint16_t warning_to_be_used_1 =  2<<warning_bit | warning;
        const uint16_t warning_to_be_used_2 =  3<<warning_bit | warning;
        const uint16_t warning_to_be_used_3 =  4<<warning_bit | warning;
        const uint16_t warning_to_be_used_4 =  5<<warning_bit | warning;
        const uint16_t warning_to_be_used_5 =  6<<warning_bit | warning;
        const uint16_t warning_to_be_used_6 =  7<<warning_bit | warning;
        const uint16_t warning_to_be_used_7 =  8<<warning_bit | warning;
        const uint16_t warning_to_be_used_8 =  9<<warning_bit | warning;
        const uint16_t warning_to_be_used_9 =  10<<warning_bit | warning;
        const uint16_t warning_to_be_used_10 =  11<<warning_bit | warning;
        const uint16_t warning_to_be_used_11 =  12<<warning_bit | warning;
        const uint16_t warning_to_be_used_12 =  13<<warning_bit | warning;
        const uint16_t warning_to_be_used_13 =  14<<warning_bit | warning;
        const uint16_t warning_to_be_used_14 =  15<<warning_bit | warning; 
    }
}

void print_status_message(uint16_t message);

#endif
