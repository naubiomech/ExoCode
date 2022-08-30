/**
 * @file SPIMessageQueue.h
 * @author Chance Cuddeback
 * @brief Defines a FIFO queue for the SPIMessage type. The maximum size is specified in the cpp. 
 * @date 2022-08-29
 * 
 */

#ifndef SPIMESSAGEQUEUE_H
#define SPIMESSAGEQUEUE_H

#include "spi_map.h"
#include "BleMessage.h"

namespace spi_queue
{
    spi_joint_pair_t pop();
    void push(BleMessage* msg);
    void push(spi_joint_pair_t msg);
    int size();
};

#endif