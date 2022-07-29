#include "Arduino.h"
#include "BleMessageQueue.h"
//For mutex lock
#if defined(ARDUINO_ARDUINO_NANO33BLE)
#include "mbed.h"
#include "rtos.h"
static rtos::Mutex queue_mutex;
#endif

static const int max_size = 10;
static BleMessage queue[max_size];
static int size = 0;
static const BleMessage empty_message = BleMessage();

BleMessage pop_queue()
{
    #if defined(ARDUINO_ARDUINO_NANO33BLE)
    queue_mutex.lock();
    #endif
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
    #if defined(ARDUINO_ARDUINO_NANO33BLE)
    queue_mutex.unlock();
    #endif
}

void push_queue(BleMessage* msg)
{
    #if defined(ARDUINO_ARDUINO_NANO33BLE)
    queue_mutex.lock();
    #endif
    if (size == (max_size-1))
    {
        Serial.println("BleMessageQueue::push_queue->Queue Full!");
        return;
    }

    size++;
    queue[size].copy(msg);
    #if defined(ARDUINO_ARDUINO_NANO33BLE)
    queue_mutex.unlock();
    #endif
}

int queue_size()
{
    return size;
}

int check_queue_for(BleMessage msg)
{
    int found = 0;
    for (int i=0; i<size; i++)
    {
        found += BleMessage::matching(queue[i], msg);
    }
    return found;
}