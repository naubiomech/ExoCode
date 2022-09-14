#ifndef UART_COMMANDS_H
#define UART_COMMANDS_H

/**
 * @brief Type to associate a command with an ammount of data
 * 
 */
typedef struct
{
    char command;
    enum class enumerator; 
} uart_map_t;

namespace uart_cmd_names
{
    static const uint8_t empty_msg = 0x00;
    static const uint8_t get_controller = 0x01;
    static const uint8_t get_controller_params = 0x02;
    static const uint8_t update_controller = 0x03;
    static const uint8_t update_controller_params = 0x04;
    static const uint8_t get_status = 0x05;
    static const uint8_t update_status = 0x06;
    static const uint8_t get_config = 0x07;
    static const uint8_t update_config = 0x08;
    static const uint8_t get_cal_trq_sensor = 0x09;
    static const uint8_t update_cal_trq_sensor = 0x0A;
    static const uint8_t get_cal_fsr = 0x0B;
    static const uint8_t update_cal_fsr = 0x0C;
    static const uint8_t get_refine_fsr = 0x0D;
    static const uint8_t update_refine_fsr = 0x0E;
    static const uint8_t get_motor_enable_disable = 0x0F;
    static const uint8_t update_motor_enable_disable = 0x010;
    static const uint8_t get_motor_zero = 0x11;
    static const uint8_t update_motor_zero = 0x12;
};

/**
 * @brief Holds all of the enums for the uart commands. The enums are used to properly index the data
 * 
 */
namespace uart_cmd_enums
{
    enum class controller {
        CONTROLLER_ID = 0,
        LENGTH
    };
    enum class controller_params {
        CONTROLLER_ID = 0,
        PARAM_START = 1,
        LENGTH
    };
    enum class status {
        STATUS = 0,
        LENGTH
    };
    enum class config {
        CONFIG_START = 0,
        LENGTH
    };
    enum class cal_trq_sensor {
        CAL_TRQ_SENSOR = 0,
        LENGTH
    };
    enum class cal_fsr {
        CAL_FSR = 0,
        LENGTH
    };
    enum class refine_fsr {
        REFINE_FSR = 0,
        LENGTH
    };
    enum class motor_enable_disable {
        ENABLE_DISABLE = 0,
        LENGTH
    };
    enum class motor_zero {
        ZERO = 0,
        LENGTH
    };
};


namespace uart_map
{
    /**
     * @brief An array defining the maps from command to ENUM. Only the update_ commands need an enum
     * class, because the get_ commands are data requests
     */
    static const uart_map_t maps[] = 
    {
        {uart_cmd_names::update_controller, uart_cmd_enums::controller},
        {uart_cmd_names::update_controller_params, uart_cmd_enums::controller_params},
        {uart_cmd_names::update_status, uart_cmd_enums::status},
        {uart_cmd_names::update_config, uart_cmd_enums::config},
        {uart_cmd_names::update_cal_trq_sensor, uart_cmd_enums::cal_trq_sensor},
        {uart_cmd_names::update_cal_fsr, uart_cmd_enums::cal_fsr},
        {uart_cmd_names::update_refine_fsr, uart_cmd_enums::refine_fsr},
        {uart_cmd_names::update_motor_enable_disable, uart_cmd_enums::motor_enable_disable},
        {uart_cmd_names::update_motor_zero, uart_cmd_enums::motor_zero},
    };
};







#endif