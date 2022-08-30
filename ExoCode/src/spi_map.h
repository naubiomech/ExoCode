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