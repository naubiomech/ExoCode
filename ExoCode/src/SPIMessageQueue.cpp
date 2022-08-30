#include "SPIMessageQueue.h"

static const int k_max_size = 10;
static spi_joint_pair_t queue[k_max_size];
static int m_size = 0;
static const spi_joint_pair_t empty_message = {0, 0};


spi_joint_pair_t spi_queue::pop()
{
    if(spi_queue::size()) 
    {
        spi_joint_pair_t msg = queue[m_size];
        m_size--;
        return msg;
    }
    else
    {
        Serial.println("BleMessageQueue::pop_queue->No messages in Queue!");
        return empty_message;
    }
}

void spi_queue::push(BleMessage* msg)
{
    spi_joint_pair_t spi_joint_pair = spi_command_helpers::get_spi_cmd_for_ble_cmd(*msg);
    spi_queue::push(spi_joint_pair);
}

void spi_queue::push(spi_joint_pair_t msg)
{
    if (m_size == (k_max_size-1))
    {
        Serial.println("SPIMessageQueue::push_queue->Queue Full!");
        return;
    }

    m_size++;
    queue[m_size] = (msg); //TODO: Test that placement is permanent
}

int spi_queue::size()
{
    return m_size;
}