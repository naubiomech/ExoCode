#if defined(ARDUINO_ARDUINO_NANO33BLE)
#include "Arduino.h"
#include "BleMessageQueue.h"
//For mutex lock
#include "mbed.h"
#include "rtos.h"
static rtos::Mutex queue_mutex;

static const int max_size = 10;
static BleMessage queue[max_size];
static int size = 0;
static const BleMessage empty_message = BleMessage();

BleMessage pop_queue()
{
    queue_mutex.lock();
    if(queue_size()) 
    {
        BleMessage msg = queue[size];
        size--;
        return msg;
    }
    else
    {
        Serial.println("BleMessageQueue::pop_queue->No messages in Queue!");
        return empty_message;
    }
    queue_mutex.unlock();
}

void push_queue(BleMessage* msg)
{
    queue_mutex.lock();

    if (size == (max_size-1))
    {
        Serial.println("BleMessageQueue::push_queue->Queue Full!");
        return;
    }

    size++;
    queue[size].copy(msg);

    queue_mutex.unlock();
}

int queue_size()
{
    return size;
}

#endif