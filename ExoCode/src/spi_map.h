#ifndef SPI_MAP_H
#define SPI_MAP_H

#include "SPIHandler.h"
#include "ble_commands.h"

typedef struct
{
    uint8_t spi_id;
    uint8_t joint_id;
} spi_joint_pair_t;

typedef struct
{
    uint8_t ble_id;
    uint8_t spi_id;
    bool use_joint;
} spi_map_t;

namespace ble_spi_mappings
{
    //TODO: populate
    static spi_map_t values[]
    {
        {ble_names::start, spi_cmd::update_status::id, 0},
        {ble_names::stop, spi_cmd::update_status::id, 0},
        {ble_names::cal_trq, spi_cmd::calibrate_torque_sensor::id, 0},// TODO: update to send all flags
        {ble_names::cal_fsr, spi_cmd::calibrate_fsr_workaround::id, 0},
        {ble_names::new_trq, spi_cmd::update_controller_params_workaround::id, 1},
        {ble_names::assist, spi_cmd::send_data_exo::id, 0},// not currently used
        {ble_names::resist, spi_cmd::send_data_exo::id, 0},// not currently used
        {ble_names::motors_on, spi_cmd::motor_enable_disable::id, 0},
        {ble_names::motors_off, spi_cmd::motor_enable_disable::id, 0},
        {ble_names::mark, spi_cmd::send_data_exo::id, 0},// not currently used.
        {ble_names::send_real_time_data, spi_cmd::send_data_exo::id, 0},
        {ble_names::send_batt, spi_cmd::send_data_exo::id, 0},// not currently used.
        {ble_names::send_cal_done, spi_cmd::send_data_exo::id, 0},// data will have cal flags
        {ble_names::send_error_count, spi_cmd::send_data_exo::id, 0},// not currently used.
        {ble_names::send_trq_cal, spi_cmd::send_data_exo::id, 0},// data will have cal flags
        {ble_names::send_step_count, spi_cmd::send_data_exo::id, 0},// not currently used.
        {ble_names::cal_fsr_finished, spi_cmd::send_data_exo::id, 0},// data will have cal flags


    };
};


namespace spi_command_helpers
{
    inline static spi_joint_pair_t get_spi_cmd_for_ble_cmd(BleMessage ble_msg)
    {
        uint8_t ble_id = ble_msg.command;
        uint8_t spi_id = 0;
        uint8_t joint_id = 0;
        
        for(int i=0; i < sizeof(ble_spi_mappings::values)/sizeof(ble_spi_mappings::values[0]); i++)
        {
            if(ble_id == ble_spi_mappings::values[i].ble_id)
            {
                spi_id = ble_spi_mappings::values[i].spi_id;
                if (ble_spi_mappings::values[i].use_joint) 
                {
                    joint_id = ble_msg.data[0];
                }
                break;
            } 
        }

        return {spi_id, joint_id};
    }
}

#endif