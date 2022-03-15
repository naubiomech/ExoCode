/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Controllerdata_h
#define ControllerData_h
#include <stdint.h>

#include "Arduino.h"

#include "Board.h"
#include "ParseIni.h"
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
        const uint8_t max_torque_idx = 0;  // parameter for peak exo torque
        const uint8_t num_parameter = 1;
    }
    
    namespace heel_toe
    {
        const uint8_t max_torque_idx = 0;
        const uint8_t min_torque_idx = 1;
        const uint8_t num_parameter = 2;
    }
    
    namespace extension_angle
    {
        // parameters for maximum exo extension and flexion torque.
        const uint8_t flexion_setpoint_idx = 0;
        const uint8_t extension_setpoint_idx = 1;
        // Parameter for fraction of peak flexion angle where mode will switch to extension angle.
        const uint8_t target_flexion_percent_max_idx = 2;
        // Parameter for flag to reset the the peak range of motion angles.
        const uint8_t clear_angle_idx = 3;
        const uint8_t num_parameter = 4;
    }
    
    namespace zhang_collins
    {
        // user mass kg
        const uint8_t mass_idx = 0;
        // peak torque divided by user mass 
        const uint8_t peak_normalized_torque_Nm_kg_idx = 1;
        // ramp start percent gait x10
        const uint8_t t0_idx = 2;
        // torque onset percent gait x10
        const uint8_t t1_idx = 3;
        // peak torque point as percent gait x10
        const uint8_t t2_idx = 4;
        // offset as percent gait x10
        const uint8_t t3_idx = 5;
        const uint8_t num_parameter = 6;
        
    }
    
    const uint8_t max_parameters = 6;  // this should be the largest of all the num_parameters
}


class ControllerData {
	public:
        ControllerData(config_defs::joint_id id, uint8_t* config_to_send);
        
        uint8_t controller;
        config_defs::JointType joint; 
        // These were made floats to dummy proof the math for people but will double the data needed to be sent over SPI, we double the speed of the SPI if we move to fixed point.
        float setpoint;  //  controller setpoint, basically the motor command.
        float parameters[controller_defs::max_parameters];  // Parameter list for the controller see the controller_defs namespace for the specific controller.  
};


#endif