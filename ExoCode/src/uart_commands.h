#ifndef UART_COMMANDS_H
#define UART_COMMANDS_H

#include "Arduino.h"
#include "UARTHandler.h"
#include "UART_msg_t.h"

#include "ParseIni.h" // for config info
#include "ExoData.h"
#include "JointData.h"
#include "ParamsFromSD.h"

/**
 * @brief Type to associate a command with an ammount of data
 * 
 */
typedef struct
{
    char command;
    enum class enumerator; 
} UART_map_t;

namespace UART_command_names
{
    /* update_x must be get_x + 1 */
    static const uint8_t empty_msg = 0x00;
    static const uint8_t get_controller_params = 0x01;
    static const uint8_t update_controller_params = 0x02;
    static const uint8_t get_status = 0x03;
    static const uint8_t update_status = 0x04;
    static const uint8_t get_config = 0x05;
    static const uint8_t update_config = 0x06;
    static const uint8_t get_cal_trq_sensor = 0x07;
    static const uint8_t update_cal_trq_sensor = 0x08;
    static const uint8_t get_cal_fsr = 0x09;
    static const uint8_t update_cal_fsr = 0x0A;
    static const uint8_t get_refine_fsr = 0x0B;
    static const uint8_t update_refine_fsr = 0x0C;
    static const uint8_t get_motor_enable_disable = 0x0D;
    static const uint8_t update_motor_enable_disable = 0x0E;
    static const uint8_t get_motor_zero = 0x0F;
    static const uint8_t update_motor_zero = 0x10;
    static const uint8_t get_real_time_data = 0x11;
    static const uint8_t update_real_time_data = 0x12;
    static const uint8_t get_controller_param = 0x13;
    static const uint8_t update_controller_param = 0x14;
};

/**
 * @brief Holds all of the enums for the UART commands. The enums are used to properly index the data
 * 
 */
namespace UART_command_enums
{
    enum class controller_params:uint8_t {
        CONTROLLER_ID = 0,
        PARAM_LENGTH = 1,
        PARAM_START = 2,
        LENGTH
    };
    enum class status:uint8_t {
        STATUS = 0,
        LENGTH
    };
    enum class cal_trq_sensor:uint8_t {
        CAL_TRQ_SENSOR = 0,
        LENGTH
    };
    enum class cal_fsr:uint8_t {
        CAL_FSR = 0,
        LENGTH
    };
    enum class refine_fsr:uint8_t {
        REFINE_FSR = 0,
        LENGTH
    };
    enum class motor_enable_disable:uint8_t {
        ENABLE_DISABLE = 0,
        LENGTH
    };
    enum class motor_zero:uint8_t {
        ZERO = 0,
        LENGTH
    };
    enum class controller_param:uint8_t {
        CONTROLLER_ID = 0,
        PARAM_INDEX = 1,
        PARAM_VALUE = 2,
        LENGTH
    };
    enum class real_time_data:uint8_t {
        
    };
};

namespace UART_rt_data 
{
    static UART_msg_t msg;
    static uint8_t msg_len;

    static const uint8_t BILATERAL_HIP_RT_LEN = 8;
    static const uint8_t BILATERAL_HIP_ANKLE_RT_LEN = 8;
    static const uint8_t BILATERAL_ANKLE_RT_LEN = 8;

    static uint8_t new_rt_msg = 0; // Flag for new message
};


namespace UART_map
{
    /**
     * @brief An array defining the maps from command to ENUM. Only the update_ commands need an enum
     * class, because the get_ commands are data requests
     */
    // static const UART_map_t maps[] = 
    // {
    //     {UART_command_names::update_controller, UART_command_enums::controller},
    //     {UART_command_names::update_controller_params, UART_command_enums::controller_params},
    //     {UART_command_names::update_status, UART_command_enums::status},
    //     {UART_command_names::update_config, UART_command_enums::config},
    //     {UART_command_names::update_cal_trq_sensor, UART_command_enums::cal_trq_sensor},
    //     {UART_command_names::update_cal_fsr, UART_command_enums::cal_fsr},
    //     {UART_command_names::update_refine_fsr, UART_command_enums::refine_fsr},
    //     {UART_command_names::update_motor_enable_disable, UART_command_enums::motor_enable_disable},
    //     {UART_command_names::update_motor_zero, UART_command_enums::motor_zero},
    //     {UART_command_names::update_real_time_data, UART_command_enums::real_time_data},
    // };
};


/**
 * @brief Holds the handlers for all of the commands. The handler function types should be the same. The 'get'
 * handlers will respond with the appropriate command, the 'update' handlers will unpack the msg and pack 
 * exo_data
 * 
 */
namespace UART_command_handlers
{
    inline static void get_controller_params(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {
        //Serial.println("UART_command_handlers::update_controller_params->Fetching params with msg: ");
        UART_msg_t_utils::print_msg(msg);

        JointData* j_data = exo_data->get_joint_with(msg.joint_id);
        if (j_data == NULL)
        {
            //Serial.println("UART_command_handlers::update_controller_params->No joint with id =  "); Serial.print(msg.joint_id); Serial.println(" found");
            return;
        }

        msg.command = UART_command_names::update_controller_params;

        uint8_t param_length = j_data->controller.get_parameter_length();
        msg.len = param_length + (uint8_t)UART_command_enums::controller_params::LENGTH; 
        msg.data[(uint8_t)UART_command_enums::controller_params::CONTROLLER_ID] = j_data->controller.controller;
        msg.data[(uint8_t)UART_command_enums::controller_params::PARAM_LENGTH] = param_length;
        for (int i=0; i<param_length; i++)
        {
            msg.data[(uint8_t)UART_command_enums::controller_params::PARAM_START + i] = j_data->controller.parameters[i];
        }

        handler->UART_msg(msg);
    }
    inline static void update_controller_params(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {
        //TODO: Error checking (valid controller for joint, and matching param length)
        //Serial.println("UART_command_handlers::update_controller_params->Got new params with msg: ");
        UART_msg_t_utils::print_msg(msg);

        JointData* j_data = exo_data->get_joint_with(msg.joint_id);
        if (j_data == NULL)
        {
            //Serial.println("UART_command_handlers::update_controller_params->No joint with id =  "); Serial.print(msg.joint_id); Serial.println(" found");
            return;
        }

        // j_data->controller.controller = msg.data[(uint8_t)UART_command_enums::controller_params::CONTROLLER_ID];
        // for (uint8_t i=0; i<msg.data[(uint8_t)UART_command_enums::controller_params::PARAM_LENGTH]; i++)
        // {
        //     Serial.print("UART_command_handlers::update_controller_params->packing ");
        //     Serial.print(msg.data[(uint8_t)UART_command_enums::controller_params::PARAM_START + i]);
        //     Serial.print(" at index ");
        //     Serial.println(i);
        //     j_data->controller.parameters[i] = msg.data[(uint8_t)UART_command_enums::controller_params::PARAM_START + i];
        // }
        #if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
        set_controller_params(msg.joint_id, (uint8_t)msg.data[(uint8_t)UART_command_enums::controller_params::CONTROLLER_ID], (uint8_t)msg.data[(uint8_t)UART_command_enums::controller_params::PARAM_START], exo_data);
        
        // get joint_data  from id
        j_data->controller.controller = (uint8_t)msg.data[(uint8_t)UART_command_enums::controller_params::CONTROLLER_ID];

        #endif
    }

    inline static void get_status(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {
        
        UART_msg_t tx_msg;
        tx_msg.command = UART_command_names::update_status;
        tx_msg.joint_id = 0;
        tx_msg.len = (uint8_t)UART_command_enums::status::LENGTH;
        tx_msg.data[(uint8_t)UART_command_enums::status::STATUS] = exo_data->status;

        handler->UART_msg(tx_msg);
        //Serial.println("UART_command_handlers::get_status->sent updated status");
    }
    inline static void update_status(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {
        //Serial.println("UART_command_handlers::update_status->got message: ");
        UART_msg_t_utils::print_msg(msg);
        exo_data->status = msg.data[(uint8_t)UART_command_enums::status::STATUS];
    }

    inline static void get_config(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {
        UART_msg_t tx_msg;
        tx_msg.command = UART_command_names::update_config;
        tx_msg.joint_id = 0;
        tx_msg.len = ini_config::number_of_keys;
        tx_msg.data[config_defs::board_name_idx] = exo_data->config[config_defs::board_name_idx];
        tx_msg.data[config_defs::battery_idx] = exo_data->config[config_defs::battery_idx];
        tx_msg.data[config_defs::board_version_idx] = exo_data->config[config_defs::board_version_idx];
        tx_msg.data[config_defs::exo_name_idx] = exo_data->config[config_defs::exo_name_idx];
        tx_msg.data[config_defs::exo_side_idx] = exo_data->config[config_defs::exo_side_idx];
        tx_msg.data[config_defs::hip_idx] = exo_data->config[config_defs::hip_idx];
        tx_msg.data[config_defs::knee_idx] = exo_data->config[config_defs::knee_idx];
        tx_msg.data[config_defs::ankle_idx] = exo_data->config[config_defs::ankle_idx];
        tx_msg.data[config_defs::hip_gear_idx] = exo_data->config[config_defs::hip_gear_idx];
        tx_msg.data[config_defs::knee_gear_idx] = exo_data->config[config_defs::knee_gear_idx];
        tx_msg.data[config_defs::ankle_gear_idx] = exo_data->config[config_defs::ankle_gear_idx];
        tx_msg.data[config_defs::exo_hip_default_controller_idx] = exo_data->config[config_defs::exo_hip_default_controller_idx];
        tx_msg.data[config_defs::exo_knee_default_controller_idx] = exo_data->config[config_defs::exo_knee_default_controller_idx];
        tx_msg.data[config_defs::exo_ankle_default_controller_idx] = exo_data->config[config_defs::exo_ankle_default_controller_idx];
        tx_msg.data[config_defs::hip_flip_dir_idx] = exo_data->config[config_defs::hip_flip_dir_idx];
        tx_msg.data[config_defs::knee_flip_dir_idx] = exo_data->config[config_defs::knee_flip_dir_idx];
        tx_msg.data[config_defs::ankle_flip_dir_idx] = exo_data->config[config_defs::ankle_flip_dir_idx];

        handler->UART_msg(tx_msg);
        //Serial.println("UART_command_handlers::get_config->sent updated config");
    }
    inline static void update_config(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {
        //Serial.println("UART_command_handlers::update_config->got message: ");
        UART_msg_t_utils::print_msg(msg);
        exo_data->config[config_defs::board_name_idx] = msg.data[config_defs::board_name_idx];
        exo_data->config[config_defs::battery_idx] = msg.data[config_defs::battery_idx];
        exo_data->config[config_defs::board_version_idx] = msg.data[config_defs::board_version_idx];
        exo_data->config[config_defs::exo_name_idx] = msg.data[config_defs::exo_name_idx];
        exo_data->config[config_defs::exo_side_idx] = msg.data[config_defs::exo_side_idx];
        exo_data->config[config_defs::hip_idx] = msg.data[config_defs::hip_idx];
        exo_data->config[config_defs::knee_idx] = msg.data[config_defs::knee_idx];
        exo_data->config[config_defs::ankle_idx] = msg.data[config_defs::ankle_idx];
        exo_data->config[config_defs::hip_gear_idx] = msg.data[config_defs::hip_gear_idx];
        exo_data->config[config_defs::knee_gear_idx] = msg.data[config_defs::knee_gear_idx];
        exo_data->config[config_defs::ankle_gear_idx] = msg.data[config_defs::ankle_gear_idx];
        exo_data->config[config_defs::exo_hip_default_controller_idx] = msg.data[config_defs::exo_hip_default_controller_idx];
        exo_data->config[config_defs::exo_knee_default_controller_idx] = msg.data[config_defs::exo_knee_default_controller_idx];
        exo_data->config[config_defs::exo_ankle_default_controller_idx] = msg.data[config_defs::exo_ankle_default_controller_idx];
        exo_data->config[config_defs::hip_flip_dir_idx] = msg.data[config_defs::hip_flip_dir_idx];
        exo_data->config[config_defs::knee_flip_dir_idx] = msg.data[config_defs::knee_flip_dir_idx];
        exo_data->config[config_defs::ankle_flip_dir_idx] = msg.data[config_defs::ankle_flip_dir_idx];
    }

    inline static void get_cal_trq_sensor(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {

    }
    inline static void update_cal_trq_sensor(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {
        //Serial.println("UART_command_handlers::update_cal_trq_sensor->Got Cal trq sensor");
        exo_data->for_each_joint([](JointData* j_data, float* args) {j_data->calibrate_torque_sensor = j_data->is_used;});
    }

    inline static void get_cal_fsr(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {

    }
    inline static void update_cal_fsr(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {
        //Serial.println("UART_command_handlers::update_cal_fsr->Got msg");
        exo_data->right_leg.do_calibration_toe_fsr = 1;    
        exo_data->right_leg.do_calibration_heel_fsr = 1;
        exo_data->left_leg.do_calibration_toe_fsr = 1;
        exo_data->left_leg.do_calibration_heel_fsr = 1;
    }

    inline static void get_refine_fsr(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {

    }
    inline static void update_refine_fsr(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {
        // TODO: only calibrate if the fsr is used
        //Serial.println("UART_command_handlers::update_refine_fsr->Got msg");
        exo_data->right_leg.do_calibration_refinement_toe_fsr = 1;
        exo_data->right_leg.do_calibration_refinement_heel_fsr = 1;
        exo_data->left_leg.do_calibration_refinement_toe_fsr = 1;
        exo_data->left_leg.do_calibration_refinement_heel_fsr = 1;
    }

    inline static void get_motor_enable_disable(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {


    }
    inline static void update_motor_enable_disable(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {
        //Serial.println("UART_command_handlers::update_motor_enable_disable->Got msg");
        exo_data->for_each_joint([](JointData* j_data, float* args) {if (j_data->is_used) j_data->motor.enabled = args[0];}, msg.data);
    }

    inline static void get_motor_zero(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {

    }
    inline static void update_motor_zero(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {


    }

    inline static void get_real_time_data(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg, uint8_t *config)
    {
        UART_msg_t rx_msg;
        rx_msg.command = UART_command_names::update_real_time_data;
        rx_msg.joint_id = 0;
        rx_msg.len = (uint8_t)UART_rt_data::BILATERAL_ANKLE_RT_LEN + 1;

        // Serial.println("config[config_defs::exo_name_idx] :: "); //Uncomment if you want to check that system is receiving correct config info
        // Serial.println(config[config_defs::exo_name_idx]);

        switch (config[config_defs::exo_name_idx])
        {
            case (uint8_t)config_defs::exo_name::bilateral_ankle:
                rx_msg.len = (uint8_t)UART_rt_data::BILATERAL_ANKLE_RT_LEN;
                rx_msg.data[0] = exo_data->right_leg.ankle.controller.filtered_torque_reading;
                rx_msg.data[1] = exo_data->right_leg.toe_stance;
                rx_msg.data[2] = exo_data->right_leg.ankle.controller.ff_setpoint; 
                rx_msg.data[3] = exo_data->left_leg.ankle.controller.filtered_torque_reading; //rx_msg.data[3] = exo_data->right_leg.ankle.motor.i;
                //TODO: Implement Mark Feature
                rx_msg.data[4] = exo_data->left_leg.toe_stance; //rx_msg.data[4] = exo_data->left_leg.toe_stance; 
                rx_msg.data[5] = exo_data->left_leg.ankle.controller.ff_setpoint;
                //rx_msg.data[6] = exo_data->right_leg.thigh_angle / 100;
                //rx_msg.data[7] = exo_data->left_leg.thigh_angle / 100;
                // rx_msg.data[6] = exo_data->right_leg.toe_fsr;
                // rx_msg.data[7] = exo_data->left_leg.toe_fsr;
                rx_msg.data[6] = exo_data->right_leg.ankle.controller.kf;
                rx_msg.data[7] = exo_data->left_leg.ankle.controller.kf;
                break;

            case (uint8_t)config_defs::exo_name::bilateral_hip:
                rx_msg.len = (uint8_t)UART_rt_data::BILATERAL_HIP_RT_LEN;
                rx_msg.data[0] = exo_data->right_leg.percent_gait / 100;
                rx_msg.data[1] = exo_data->right_leg.toe_stance;
                rx_msg.data[2] = exo_data->right_leg.hip.controller.setpoint;
                rx_msg.data[3] = exo_data->left_leg.percent_gait / 100;
                rx_msg.data[4] = exo_data->left_leg.toe_stance;  
                rx_msg.data[5] = exo_data->left_leg.hip.controller.setpoint;
                rx_msg.data[6] = exo_data->right_leg.toe_fsr;
                rx_msg.data[7] = exo_data->left_leg.toe_fsr;
                break;

            case (uint8_t)config_defs::exo_name::bilateral_hip_ankle:
                rx_msg.len = (uint8_t)UART_rt_data::BILATERAL_ANKLE_RT_LEN;
                rx_msg.data[0] = exo_data->right_leg.ankle.controller.filtered_torque_reading;
                rx_msg.data[1] = exo_data->right_leg.hip.controller.setpoint;
                rx_msg.data[2] = exo_data->right_leg.ankle.controller.ff_setpoint;
                rx_msg.data[3] = exo_data->left_leg.ankle.controller.filtered_torque_reading; //rx_msg.data[3] = exo_data->right_leg.ankle.motor.i; //
                //TODO: Implement Mark Feature
                rx_msg.data[4] = exo_data->left_leg.hip.controller.setpoint; //rx_msg.data[4] = exo_data->left_leg.toe_stance; 
                rx_msg.data[5] = exo_data->left_leg.ankle.controller.ff_setpoint;
                rx_msg.data[6] = exo_data->right_leg.toe_fsr;
                rx_msg.data[7] = exo_data->left_leg.toe_fsr;
                break;
                break;
            
            default:
                rx_msg.len = (uint8_t)UART_rt_data::BILATERAL_ANKLE_RT_LEN;
                rx_msg.data[0] = exo_data->right_leg.ankle.controller.filtered_torque_reading;
                rx_msg.data[1] = exo_data->right_leg.toe_stance;
                rx_msg.data[2] = exo_data->right_leg.ankle.controller.ff_setpoint;
                rx_msg.data[3] = exo_data->left_leg.ankle.controller.filtered_torque_reading;
                //TODO: Implement Mark Feature
                rx_msg.data[4] = exo_data->left_leg.toe_stance; 
                rx_msg.data[5] = exo_data->left_leg.ankle.controller.ff_setpoint;
                rx_msg.data[6] = exo_data->right_leg.toe_fsr;
                rx_msg.data[7] = exo_data->left_leg.toe_fsr;
                break;
        }

        handler->UART_msg(rx_msg);
        //Serial.println("UART_command_handlers::get_real_time_data->sent real time data");   Uncomment if you want to test to see what data is being sent
        //UART_msg_t_utils::print_msg(rx_msg);
    }

    // Overload for no config
    inline static void get_real_time_data(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {
        uint8_t empty_config[ini_config::number_of_keys] = {0};
        get_real_time_data(handler, exo_data, msg, empty_config);
    }


    inline static void update_real_time_data(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {
        // Serial.println("UART_command_handlers::update_real_time_data->got message: ");
        // UART_msg_t_utils::print_msg(msg);
        UART_rt_data::msg.len = msg.len;
        for (int i = 0; i < msg.len; i++)
        {
            UART_rt_data::msg.data[i] = msg.data[i];
        }
        UART_rt_data::new_rt_msg = true;
        // exo_data->right_leg.ankle.torque_reading = msg.data[0];
        // exo_data->right_leg.ankle.controller.setpoint = msg.data[2];
        // exo_data->left_leg.ankle.torque_reading = msg.data[3];
        // exo_data->left_leg.ankle.controller.setpoint = msg.data[5];
        // exo_data->right_leg.toe_fsr = msg.data[6];
        // exo_data->left_leg.toe_fsr = msg.data[7];
    }

    inline static void update_controller_param(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {
        // Serial.println("UART_command_handlers::update_controller_param->got message: ");
        // UART_msg_t_utils::print_msg(msg);
        // Get the joint
        JointData* j_data = exo_data->get_joint_with(msg.joint_id);
        if (j_data == NULL)
        {
            Serial.println("UART_command_handlers::update_controller_params->No joint with id =  "); Serial.print(msg.joint_id); Serial.println(" found");
            return;
        }
        // TODO: If the controller is different, set the default controller params. Maybe reset the joint? Should be done with a helper function 
        // Set the controller
        j_data->controller.controller = msg.data[(uint8_t)UART_command_enums::controller_param::CONTROLLER_ID];
        // Set the parameter
        j_data->controller.parameters[(uint8_t)msg.data[(uint8_t)UART_command_enums::controller_param::PARAM_INDEX]] = msg.data[(uint8_t)UART_command_enums::controller_param::PARAM_VALUE];
    }
};


namespace UART_command_utils
{

    static UART_msg_t call_and_response(UARTHandler* handler, UART_msg_t msg, float timeout)
    {
        UART_msg_t rx_msg = {0, 0, 0, 0};
        uint8_t searching = 1;
        // Serial.println("UART_command_utils::call_and_response->searching for message");
        float start_time = millis();
        while (searching)
        {
            handler->UART_msg(msg);
            // Serial.println("UART_command_utils::call_and_response->sent msg");
            delay(500);
            rx_msg = handler->poll(200000);
            searching = (rx_msg.command != (msg.command+1));
            // TODO add timeout
            if (millis() - start_time > timeout)
            {
                // Serial.println("UART_command_utils::call_and_response->timed out");
                return rx_msg;
            }
        }
        // Serial.println("UART_command_utils::call_and_response->found message:");
        UART_msg_t_utils::print_msg(rx_msg);
        return rx_msg;
    }

    static uint8_t get_config(UARTHandler* handler, uint8_t* config, float timeout)
    {
        UART_msg_t msg;
        float start_time = millis();
        while (1)
        {
            msg.command = UART_command_names::get_config;
            msg.len = 0;
            msg = call_and_response(handler, msg, timeout);

            if ((millis() - start_time) > timeout)
            {
                // Serial.println("UART_command_utils::get_config->timed out");
                return 1;
            }

            // the length of the message needs to be equal to the config length
            if (msg.len != ini_config::number_of_keys)
            {
                // Serial.println("UART_command_utils::get_config->msg.len != number_of_keys");
                // keep trying to get config
                continue;
            }
            for (int i=0; i<msg.len; i++)
            {
                // a valid config will not contain a zero
                if (!msg.data[i]) 
                {
                    // Serial.print("UART_command_utils::get_config->Config contained a zero at index ");
                    // Serial.println(i);

                    // keep trying to get config
                    continue;
                }
            }
            // Serial.println("UART_command_utils::get_config->got good config");
            break;
        }

        // pack config 
        for (int i=0; i<msg.len; i++)
        {
            config[i] = msg.data[i];
        }
        return 0;
    }

    static void wait_for_get_config(UARTHandler* handler, ExoData* data, float timeout)
    {
        UART_msg_t rx_msg;
        float start_time = millis();
        while (true)
        {
            //Serial.println("UART_command_utils::wait_for_config->Polling for config");
            rx_msg = handler->poll(100000);
            if (rx_msg.command == UART_command_names::get_config)
            {
                //Serial.println("UART_command_utils::wait_for_config->Got config request");
                UART_command_handlers::get_config(handler, data, rx_msg);
                break;
            }
            delayMicroseconds(500);

            if ((millis() - start_time) > timeout)
            {
                //Serial.println("UART_command_utils::wait_for_config->Timed out");
                return;
            }
        }
        //Serial.println("UART_command_utils::wait_for_config->Sent config");
    }

    

    static void handle_msg(UARTHandler* handler, ExoData* exo_data, UART_msg_t msg)
    {
        switch (msg.command)
        {
        case UART_command_names::empty_msg:
            Serial.println("UART_command_utils::handle_message->Empty Message!");
            break;
        
        case UART_command_names::get_controller_params:
            UART_command_handlers::get_controller_params(handler, exo_data, msg);
            break;
        case UART_command_names::update_controller_params:
            UART_command_handlers::update_controller_params(handler, exo_data, msg);
            break;

        case UART_command_names::get_status:
            UART_command_handlers::get_status(handler, exo_data, msg);
            break;
        case UART_command_names::update_status:
            UART_command_handlers::update_status(handler, exo_data, msg);
            break;

        case UART_command_names::get_config:
            UART_command_handlers::get_config(handler, exo_data, msg);
            break;
        case UART_command_names::update_config:
            UART_command_handlers::update_config(handler, exo_data, msg);
            break;
        
        case UART_command_names::get_cal_trq_sensor:
            UART_command_handlers::get_cal_trq_sensor(handler, exo_data, msg);
            break;
        case UART_command_names::update_cal_trq_sensor:
            UART_command_handlers::update_cal_trq_sensor(handler, exo_data, msg);
            break;

        case UART_command_names::get_cal_fsr:
            UART_command_handlers::get_cal_fsr(handler, exo_data, msg);
            break;
        case UART_command_names::update_cal_fsr:
            UART_command_handlers::update_cal_fsr(handler, exo_data, msg);
            break;
        
        case UART_command_names::get_refine_fsr:
            UART_command_handlers::get_refine_fsr(handler, exo_data, msg);
            break;
        case UART_command_names::update_refine_fsr:
            UART_command_handlers::update_refine_fsr(handler, exo_data, msg);
            break;
        
        case UART_command_names::get_motor_enable_disable:
            UART_command_handlers::get_motor_enable_disable(handler, exo_data, msg);
            break;
        case UART_command_names::update_motor_enable_disable:
            UART_command_handlers::update_motor_enable_disable(handler, exo_data, msg);
            break;

        case UART_command_names::get_motor_zero:
            UART_command_handlers::get_motor_zero(handler, exo_data, msg);
            break;
        case UART_command_names::update_motor_zero:
            UART_command_handlers::update_motor_zero(handler, exo_data, msg);
            break;

        case UART_command_names::get_real_time_data:
            UART_command_handlers::get_real_time_data(handler, exo_data, msg);
            break;
        case UART_command_names::update_real_time_data:
            UART_command_handlers::update_real_time_data(handler, exo_data, msg);
            break;

        case UART_command_names::update_controller_param:
            UART_command_handlers::update_controller_param(handler, exo_data, msg);
            break;
        
        default:
            Serial.println("UART_command_utils::handle_message->Unknown Message!");
            UART_msg_t_utils::print_msg(msg);
            break;
        }
    }
};






#endif