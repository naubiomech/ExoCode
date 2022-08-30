/**
 * @file ble_commands.h
 * @author Chance Cuddeback
 * @brief This file declares the BLE commands, and the functions that should be called when they are received. 
 * @date 2022-08-22
 * 
 */

#ifndef BLE_COMMANDS_H
#define BLE_COMMANDS_H

#include "ExoData.h"
#include "ParseIni.h" // For config_defs
#include "StatusDefs.h" // For ExoDataStatus_t
#include "BleMessage.h"
#include "ParamsFromSD.h"

/**
 * @brief Type to associate a command with an ammount of data
 * 
 */
typedef struct
{
    char command;
    int length; 
} ble_command_t;

/**
 * @brief Creates a variable for each command value
 * 
 */
namespace ble_names
{
    // Recieved Commands
    static const char start             = 'E';
    static const char stop              = 'G';
    static const char cal_trq           = 'H';
    static const char cal_fsr           = 'L';
    static const char new_trq           = 'F';
    static const char new_fsr           = 'R';
    static const char assist            = 'c';
    static const char resist            = 'S';
    static const char motors_on         = 'x';
    static const char motors_off        = 'w';
    static const char mark              = 'N';

    // Sending Commands
    static const char send_real_time_data = '?';
    static const char send_batt           = '~';
    static const char send_cal_done       = 'n';
    static const char send_error_count    = 'w';
    static const char send_trq_cal        = 'H';
    static const char send_step_count     = 's';
    static const char cal_fsr_finished    = 'n';
};



/**
 * @brief Associates the command and ammount of data that it expects to be sent/received
 * 
 */
namespace ble
{
    static const ble_command_t commands[] = 
    {
        // Recieved Commands
        {ble_names::start,              0},
        {ble_names::stop,               0},
        {ble_names::cal_trq,            0},
        {ble_names::cal_fsr,            0},
        {ble_names::assist,             0},
        {ble_names::resist,             0},
        {ble_names::motors_on,          0},
        {ble_names::motors_off,         0},
        {ble_names::mark,               0},
        {ble_names::new_fsr,            2},
        {ble_names::new_trq,            4},
        
        // Sending Commands
        {ble_names::send_batt,              1},
        {ble_names::send_real_time_data,    9},
        {ble_names::send_error_count,       1},
        {ble_names::send_cal_done,          0},
        {ble_names::send_trq_cal,           2},
        {ble_names::send_step_count,        2},
        {ble_names::cal_fsr_finished,       0},
    };
};

/**
 * @brief Helper function(s) to be used with the command array
 * 
 */
namespace ble_command_helpers
{
    /**
     * @brief Get the ammount of data a command is expecting
     * 
     * @param command command to get the length
     * @return int Ammount of data for a command, -1 if command not found
     */
    inline static int get_length_for_command(char command)
    {
        int length = -1;
        //Get the ammount of characters to wait for
        for(int i=0; i < sizeof(ble::commands)/sizeof(ble::commands[0]); i++)
        {
            if(command == ble::commands[i].command)
            {
                length = ble::commands[i].length;
                break;
            }
        }
        return length;
    }


}

/**
 * @brief Variables used by the Handlers to track state
 * 
 */
namespace ble_handler_vars
{
    // Should be used sparingly, we chose to do this so that ExoData wasn't needlessly populated with variables
    static const uint8_t k_max_joints = 6;
    static uint8_t prev_controllers[k_max_joints] = {0, 0, 0, 0, 0, 0};

}

/**
 * @brief Holds the functions that should be called when a command is received. All command handlers should have 
 * static linkage, return void, and accept a pointer to ExoData.
 * ie "inline static void my_handler(ExoData* data, BleMessage* msg)"
 * 
 */
namespace ble_handlers
{
    inline static void start(ExoData* data, BleMessage* msg)
    {
        // Start the trial (ie Enable motors and begin streaming data)
        // if the joint is used; enable the motor, and set the controller to zero torque
        data->for_each_joint(
            // This is a lamda or anonymous function, see https://www.learncpp.com/cpp-tutorial/introduction-to-lambdas-anonymous-functions/
            [](JointData* j_data)
            {
                if (j_data->is_used)
                {
                    j_data->motor.enabled = 1;
                }
                return;
            }
        );

        // Set the data status to running
        data->status = status_defs::messages::trial_on;
    }
    inline static void stop(ExoData* data, BleMessage* msg)
    {
        // Stop the trial (inverse of start)
        // Send trial summary data (step information)
        data->for_each_joint(
            // This is a lamda or anonymous function, see https://www.learncpp.com/cpp-tutorial/introduction-to-lambdas-anonymous-functions/
            [](JointData* j_data)
            {
                if (j_data->is_used)
                {
                    j_data->motor.enabled = 0;
                }
                return;
            }
        );

        // Set the data status to off
        data->status = status_defs::messages::trial_off;
    }
    inline static void cal_trq(ExoData* data, BleMessage* msg)
    {   
        Serial.println("Calibrating Torque!");
        // Raise cal_trq flag for all joints being used, (Out of context: Should send calibration info upon cal completion)
        data->for_each_joint([](JointData* j_data) {j_data->calibrate_torque_sensor = j_data->is_used;});
    }
    inline static void cal_fsr(ExoData* data, BleMessage* msg)
    {
        // Raise cal_fsr flag, send fsr finished flag when done
        data->right_leg.do_calibration_toe_fsr = 1;
        data->right_leg.do_calibration_refinement_toe_fsr = 1;
        data->right_leg.do_calibration_heel_fsr = 1;
        data->right_leg.do_calibration_refinement_heel_fsr = 1;

        data->left_leg.do_calibration_toe_fsr = 1;
        data->left_leg.do_calibration_refinement_toe_fsr = 1;
        data->left_leg.do_calibration_heel_fsr = 1;
        data->left_leg.do_calibration_refinement_heel_fsr = 1;
    }
    inline static void assist(ExoData* data, BleMessage* msg)
    {
        // Change PJMC parameter to assist
        // Need to implement PJMC
    }
    inline static void resist(ExoData* data, BleMessage* msg)
    {
        // Change PJMC parameter to resist
        // Need to implement PJMC
    }
    inline static void motors_on(ExoData* data, BleMessage* msg)
    {
        // Enable Motors, stateless (ie keep running fault detection algorithms)
        // int count = 0;
        // data->for_each_joint(
        //     [&count, &(ble_handler_vars::prev_controllers)](JointData* j_data)
        //     {
        //         j_data->controller.controller = ble_handler_vars::prev_controllers[count];
        //         count++;
        //     }
        // );
    }
    inline static void motors_off(ExoData* data, BleMessage* msg)
    {
        // Disable Motors, stateless (ie keep running fault detection algorithms)
        // Chnage to stasis and save the previous controller
        // int count = 0;
        // data->for_each_joint(
        //     [&count, &(ble_handler_vars::prev_controllers)](JointData* j_data)
        //     {
        //         ble_handler_vars::prev_controllers[count] = (uint8_t)j_data->controller.controller;
        //         count++;

        //         j_data->controller.controller = (uint8_t)config_defs::ankle_controllers::stasis;
        //     }
        // );
    }
    inline static void mark(ExoData* data, BleMessage* msg)
    {
        // Increment mark variable (Done by sending different data on one of the real time signals, we should raise a flag or inc a var in exo_data)
    }
    inline static void new_trq(ExoData* data, BleMessage* msg)
    {
        // Serial.print("Ankle ID: "); Serial.println((uint8_t)data->left_leg.ankle.id);
        // Serial.println("Got New Trq:");
        // Serial.print(msg->data[0]); Serial.print("\t");
        // Serial.print(msg->data[1]); Serial.print("\t");
        // Serial.print(msg->data[2]); Serial.print("\t");
        // Serial.print(msg->data[3]); Serial.print("\t\r\n");
        // (LSP, LDSP, RSP, RDSP) Unpack message data
        config_defs::joint_id joint_id = (config_defs::joint_id)msg->data[0];
        uint8_t controller_id = (uint8_t)msg->data[1];
        uint8_t set_num = (uint8_t)msg->data[2];
        // Update Exo_Data controller for each joint
        ControllerData* cont_data = NULL;

        // Map the joint IDs because the GUI limits the maximum number for the message
        joint_id = (joint_id==(config_defs::joint_id)1)?(data->left_leg.hip.id):(joint_id);
        joint_id = (joint_id==(config_defs::joint_id)2)?(data->left_leg.knee.id):(joint_id);
        joint_id = (joint_id==(config_defs::joint_id)3)?(data->left_leg.ankle.id):(joint_id);
        joint_id = (joint_id==(config_defs::joint_id)4)?(data->right_leg.hip.id):(joint_id);
        joint_id = (joint_id==(config_defs::joint_id)5)?(data->right_leg.knee.id):(joint_id);
        joint_id = (joint_id==(config_defs::joint_id)6)?(data->right_leg.ankle.id):(joint_id);

        if (joint_id == data->left_leg.ankle.id) {
            cont_data = &data->left_leg.ankle.controller;
        } else if (joint_id == data->left_leg.knee.id) {
            cont_data = &data->left_leg.knee.controller;
        } else if (joint_id == data->left_leg.hip.id) {
            cont_data = &data->left_leg.hip.controller;
        } else if (joint_id == data->right_leg.ankle.id) {
            cont_data = &data->right_leg.ankle.controller;
        } else if (joint_id == data->right_leg.knee.id) {
            cont_data = &data->right_leg.knee.controller;
        } else if (joint_id == data->right_leg.hip.id) {
            cont_data = &data->right_leg.hip.controller;
        }
        if (cont_data == NULL) {
            Serial.println("cont_data is NULL!");
        }
        if (cont_data != NULL) 
        {
            cont_data->controller = controller_id;
        }

        //set_controller_params((uint8_t)joint_id, controller_id, set_num, data);
    }
    inline static void new_fsr(ExoData* data, BleMessage* msg)
    {
        // Change PJMC fsr threshold parameters
        // Need to implement PJMC

    }

}

#endif