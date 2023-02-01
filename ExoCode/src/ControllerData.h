/**
 * @file ControllerData.h
 *
 * @brief Declares a class used to store data for controllers to access 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
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


namespace controller_defs /**< stores the parameter indexes for different controllers */
{
    namespace zero_torque
    {
        const uint8_t use_pid_idx = 0;
        const uint8_t p_gain_idx = 1;
        const uint8_t i_gain_idx = 2;
        const uint8_t d_gain_idx = 3;
        const uint8_t num_parameter = 4;
    }
    
    namespace stasis
    {
        const uint8_t num_parameter = 0;
    }
    
    namespace proportional_joint_moment
    {
        const uint8_t stance_max_idx = 0;  // parameter for peak exo torque during stance
        const uint8_t swing_max_idx = 1;  // parameter for peak exo torque during swing
        const uint8_t is_assitance_idx = 2;
        const uint8_t use_pid_idx = 3;
        const uint8_t p_gain_idx = 4;
        const uint8_t i_gain_idx = 5;
        const uint8_t d_gain_idx = 6;
        const uint8_t torque_alpha_idx = 7;
        const uint8_t num_parameter = 8;
    }
    
    namespace heel_toe
    {
        const uint8_t flexion_torque_setpoint_idx = 0;
        const uint8_t extension_torque_setpoint_idx = 1;
        const uint8_t use_pid_idx = 2;
        const uint8_t p_gain_idx = 3;
        const uint8_t i_gain_idx = 4;
        const uint8_t d_gain_idx = 5;
        const uint8_t num_parameter = 6;
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
        // angle where the system will switch from extension assistance to flexion assistance
        const uint8_t angle_threshold_idx = 4;
        // Velocity where the system will switch from flexion assistance to extension assistance, if velocity inverts before the target_flexion_percent_max
        // !! Value should be NEGATIVE !!
        const uint8_t velocity_threshold_idx = 5; 
        const uint8_t use_pid_idx = 6;
        const uint8_t p_gain_idx = 7;
        const uint8_t i_gain_idx = 8;
        const uint8_t d_gain_idx = 9;
        const uint8_t num_parameter = 10;
    }
    
    namespace bang_bang
    {
        // parameters for maximum exo extension and flexion torque.
        const uint8_t flexion_setpoint_idx = 0;
        const uint8_t extension_setpoint_idx = 1;
        // Parameter for fraction of peak flexion angle where mode will switch to extension angle.
        const uint8_t is_assitance_idx = 2;
        // Parameter for fraction of peak flexion angle where mode will switch to extension angle.
        const uint8_t target_flexion_percent_max_idx = 3;
        // Parameter for flag to reset the the peak range of motion angles.
        const uint8_t clear_angle_idx = 4;
        // angle where the system will switch from extension assistance to flexion assistance
        const uint8_t angle_threshold_idx = 5;
        // Velocity where the system will switch from flexion assistance to extension assistance, if velocity inverts before the target_flexion_percent_max
        // !! Value should be NEGATIVE !!
        const uint8_t velocity_threshold_idx = 6; 
        const uint8_t use_pid_idx = 7;
        const uint8_t p_gain_idx = 8;
        const uint8_t i_gain_idx = 9;
        const uint8_t d_gain_idx = 10;
        const uint8_t num_parameter = 11;
    }

    namespace late_stance
    {
        // parameters for late stance torque.
        const uint8_t resistance_setpoint_idx = 0;
        // Parameter for fraction of peak flexion angle where mode will switch to extension angle.
        const uint8_t angle_on_off = 1;
        // Parameter for flag to reset the the peak range of motion angles.
        const uint8_t clear_angle_idx = 2;
        // Velocity where the system will switch from on to off,
        // !! Value should be NEGATIVE !!
        const uint8_t velocity_threshold_idx = 3;
        const uint8_t use_pid_idx = 4;
        const uint8_t p_gain_idx = 5;
        const uint8_t i_gain_idx = 6;
        const uint8_t d_gain_idx = 7;
        const uint8_t num_parameter = 8;
    }

    namespace gait_phase
    {
        // parameters for gait phase torque.
        const uint8_t flexion_setpoint_idx = 0;
        const uint8_t extension_setpoint_idx = 1;
        // Parameter for fraction of peak flexion angle where mode will switch to extension angle.
        const uint8_t flexion_start_percentage_idx = 2;
        const uint8_t flexion_end_percentage_idx = 3;
        const uint8_t extension_start_percentage_idx = 4;
        const uint8_t extension_end_percentage_idx = 5;
        //Parameter to set slope of line between each point (for "smoothing" purposes) 
        const uint8_t slope_idx = 6;
        //Flag for PID contorl and associated gains
        const uint8_t use_pid_idx = 7;
        const uint8_t p_gain_idx = 8;
        const uint8_t i_gain_idx = 9;
        const uint8_t d_gain_idx = 10;
        const uint8_t num_parameter = 11;
    }

    namespace parabolic
    {
        // parameters for gait phase torque.
        const uint8_t flexion_setpoint_idx = 0;
        const uint8_t extension_setpoint_idx = 1;
        const uint8_t flexion_start_percentage_idx = 2;
        const uint8_t flexion_end_percentage_idx = 3;
        const uint8_t extension_start_percentage_idx = 4;
        const uint8_t extension_end_percentage_idx = 5;
        const uint8_t num_parameter = 6;
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
        const uint8_t use_pid_idx = 6;
        const uint8_t p_gain_idx = 7;
        const uint8_t i_gain_idx = 8;
        const uint8_t d_gain_idx = 9;
        const uint8_t num_parameter = 10;
    }

    namespace franks_collins_hip
    {
        const uint8_t mass_idx = 0;                             //Mass of the User in kg
        const uint8_t trough_normalized_torque_Nm_kg_idx = 1;   //Extension Torque in Nm/kg
        const uint8_t peak_normalized_torque_Nm_kg_idx = 2;     //Flexion Torque in Nm/kg
        const uint8_t start_percent_gait_idx = 3;               //Percent of Gait Cycle where the curve starts (does not start at 0 so that there is no discontinuity)
        const uint8_t trough_onset_percent_gait_idx = 4;        //Percent of Gait Cycle where curve for extension torque starts
        const uint8_t trough_percent_gait_idx = 5;              //Percent of Gait Cycle where peak extension torque occurs
        const uint8_t mid_time_idx = 6;                         //Time from when curve starts until midpoint of zero torque period between extension and flexion torques
        const uint8_t mid_duration_idx = 7;                     //Time of zero torque period between extension and flexion torques
        const uint8_t peak_percent_gait_idx = 8;                //Percent of Gait Cycle where peak flexion torque occurs
        const uint8_t peak_offset_percent_gait_idx = 9;         //Percent of Gait Cycle where torque stops being applied
        const uint8_t use_pid_idx = 10;                         //Flag to determine whether or not PID used
        const uint8_t p_gain_idx = 11;                          //Value of P Gain for PID control
        const uint8_t i_gain_idx = 12;                          //Value of I Gain for PID control
        const uint8_t d_gain_idx = 13;                          //Value of D Gain for PID control 
        const uint8_t num_parameter = 14;                   
    }
    
    // namespace user_defined
    // {
        // const uint8_t num_sample_points = 50;  // not an index
        
        // const uint8_t mass_idx = 0; 
        // const uint8_t use_pid_idx = 1;
        // const uint8_t p_gain_idx = 2;
        // const uint8_t i_gain_idx = 3;
        // const uint8_t d_gain_idx = 4;
        // const uint8_t curve_start_idx = 5;
        // const uint8_t curve_stop_idx = curve_start_idx+num_sample_points;
        // const uint8_t num_parameter = curve_stop_idx+1;
    // }
    
    namespace sine
    {
        const uint8_t amplitude_idx = 0;    // amplitude in Nm
        const uint8_t period_idx = 1;       // period in ms
        const uint8_t phase_shift_idx = 2;  // phase shift in rad
        const uint8_t use_pid_idx = 3;
        const uint8_t p_gain_idx = 4;
        const uint8_t i_gain_idx = 5;
        const uint8_t d_gain_idx = 6;
        const uint8_t num_parameter = 7;
    }

    namespace perturbation
    {
        const uint8_t amplitude_idx = 0;    // amplitude in Nm
        const uint8_t duration_idx = 1;     // time that amplitude is applied in seconds
        const uint8_t direction_idx = 2;    // direction (0 = plantarflexion, 1 = dorsiflexion)
        const uint8_t perturb_idx = 3;      //Flag indicating whether or not the perturbation should be sent (0 = no, 1 = yes)
        const uint8_t num_parameter = 4;
            ;
    }

    namespace constant_torque
    {
        const uint8_t amplitude_idx = 0;    // amplitude in Nm
        const uint8_t direction_idx = 1;     // direction in which the torque will be applied
        const uint8_t num_parameter = 2;
        ;
    }
    /*
     * Add phase delayed controller for hip
     * peak and trough magnitude defined.
     * Delay based on percent gait.
     *
     */
    
    const uint8_t max_parameters = franks_collins_hip::num_parameter;//user_defined::num_parameter;  // this should be the largest of all the num_parameters
}

/**
 * @brief class to store information related to controllers.
 * 
 */
class ControllerData {
	public:
        ControllerData(config_defs::joint_id id, uint8_t* config_to_send);
        
        /**
         * @brief reconfigures the the controller data if the configuration changes after constructor called.
         * 
         * @param configuration array
         */
        void reconfigure(uint8_t* config_to_send);

        /**
         * @brief Get the parameter length for the current controller
         * 
         * @return uint8_t parameter length 
         */
        uint8_t get_parameter_length();
        
        
        uint8_t controller; /**< id of the current controller */
        config_defs::JointType joint; /**< id of the current joint */
        // These were made floats to dummy proof the math for people but will double the data needed to be sent over SPI, we double the speed of the SPI if we move to fixed point.
        float setpoint;  /**< controller setpoint, basically the motor command. */
        float ff_setpoint; /**< feed forwared setpoint, only updated in closed loop controllers */
        float parameters[controller_defs::max_parameters];  /**< Parameter list for the controller see the controller_defs namespace for the specific controller. */
        uint8_t parameter_set; /**< temporary value used to store the parameter set while we are pulling from the sd card. */

        float filtered_torque_reading; /**< filtered torque reading, used for filtering torque signal */
        float filtered_cmd; /**< filtered command, used for filtering motor commands */
        float filtered_setpoint = 0; /**< filtered setpoint for the controller */
        
        // Variables for Auto Kf in the PID Controller
        float kf = 1; /**< gain for the controller */
        float prev_max_measured = 0; /**< previous max measured value */
        float prev_max_setpoint = 0; /**< previous max setpoint value */
        float max_measured = 0; /**< max measured value */
        float max_setpoint = 0; /**< max setpoint value */
};


#endif