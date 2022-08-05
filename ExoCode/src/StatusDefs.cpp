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


#include "StatusDefs.h"


void print_status_message(uint16_t message)
{
    switch (message)
    {
        case status_defs::messages::off :
            Serial.print("Off");
            break;    
        case status_defs::messages::trial_off :
            Serial.print("Trial Off");
            break;
        case status_defs::messages::trial_on :
            Serial.print("Trial On");
            break;
        case status_defs::messages::test :
            Serial.print("Test");
            break;    
        case status_defs::messages::torque_calibration :
            Serial.print("Torque Calibration");
            break;
        case status_defs::messages::fsr_calibration :
            Serial.print("FSR Calibration");
            break;
        case status_defs::messages::fsr_refinement :
            Serial.print("FSR Refinement");
            break;    
        case status_defs::messages::error :
            Serial.print("General Error");
            break;
        case status_defs::messages::error_left_heel_fsr :
            Serial.print("Error :: Left Heel FSR");
            break;
        case status_defs::messages::error_left_toe :
            Serial.print("Error :: Left Toe FSR");
            break;
        case status_defs::messages::error_right_heel_fsr :
            Serial.print("Error :: Right Heel FSR");
            break;
        case status_defs::messages::error_right_toe_fsr :
            Serial.print("Error :: Right Toe FSR");
            break;    
        case status_defs::messages::error_left_hip_torque_sensor :
            Serial.print("Error :: Left Hip Torque Sensor");
            break;
        case status_defs::messages::error_left_knee_torque_sensor :
            Serial.print("Error :: Left Knee Torque Sensor");
            break;
        case status_defs::messages::error_left_ankle_torque_sensor :
            Serial.print("Error :: Left Ankle Torque Sensor");
            break;    
        case status_defs::messages::error_right_hip_torque_sensor :
            Serial.print("Error :: Right Hip Torque Sensor");
            break;
        case status_defs::messages::error_right_knee_torque_sensor :
            Serial.print("Error :: Right Knee Torque Sensor");
            break;
        case status_defs::messages::error_right_ankle_torque_sensor :
            Serial.print("Error :: Right Ankle Torque Sensor");
            break;
        case status_defs::messages::error_left_hip_motor :
            Serial.print("Error :: Left Hip Motor");
            break;
        case status_defs::messages::error_left_knee_motor :
            Serial.print("Error :: Left Knee Motor");
            break;    
        case status_defs::messages::error_left_ankle_motor :
            Serial.print("Error :: Left Ankle Motor");
            break;
        case status_defs::messages::error_right_hip_motor :
            Serial.print("Error :: Right Hip Motor");
            break;
        case status_defs::messages::error_right_knee_motor :
            Serial.print("Error :: Right Knee Motor");
            break; 
        case status_defs::messages::error_right_ankle_motor :
            Serial.print("Error :: Right Ankle Motor");
            break;
        case status_defs::messages::error_left_hip_controller :
            Serial.print("Error :: Left Hip Controller");
            break;
        case status_defs::messages::error_left_knee_controller :
            Serial.print("Error :: Left Knee Controller");
            break;
        case status_defs::messages::error_left_ankle_controller :
            Serial.print("Error :: Left Ankle Controller");
            break;
        case status_defs::messages::error_right_hip_controller :
            Serial.print("Error :: Right Hip Controller");
            break;    
        case status_defs::messages::error_right_knee_controller :
            Serial.print("Error :: Right Knee Controller");
            break;
        case status_defs::messages::error_right_ankle_controller :
            Serial.print("Error :: Right Ankle Controller");
            break;
    }
};
