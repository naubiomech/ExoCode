#if defined(ARDUINO_ARDUINO_NANO33BLE)
#include "Arduino.h"
#include "BleMessageQueue.h"

static const int max_size = 10;
static BleMessage queue[max_size];
static int size = 0;
static const BleMessage empty_message = BleMessage();

BleMessage pop_queue()
{
    Serial.println("BleMessageQueue::pop_queue->Popped");
    for (int i=0; i<size; i++)
    {
        Serial.println(i);
        Serial.println(queue[i].command);
        Serial.println(queue[i].expecting);
        for (int j=0; j<queue[i].expecting; j++)
        {
            Serial.print(queue[i].data[j]);
        }
        Serial.println();
    }

    if(queue_size()) 
    {
        BleMessage msg = queue[size];
        std::vector<float> o_data = msg.data;
        Serial.println("BleMessageQueue::pop_queue->Popped");
        Serial.print(msg.command);
        Serial.print("\t");
        Serial.println(msg.expecting);
        for (int i = 0; i < (msg.expecting); i++)
        {
            Serial.print(msg.data[i]);
            Serial.print("\t");
        }
        Serial.println();
        size--;
        return msg;
    }
    else
    {
        return empty_message;
    }
}

void push_queue(BleMessage* msg)
{
    if (size == (max_size-1))
    {
        return;
    }
    Serial.println("BleMessageQueue::push_queue->Got Message");
    Serial.print(msg->command);
    Serial.print("\t");
    Serial.println(msg->expecting);
    for (int i = 0; i < (msg->expecting); i++)
    {
        Serial.print(msg->data[i]);
        Serial.print("\t");
    }
    Serial.println();

    size++;
    queue[size].copy(msg);

    for (int i=0; i<queue[size].expecting; i++)
    {
        Serial.println(msg->data[i]);
        queue[size].data[i] = msg->data[i];
        Serial.println(queue[size].data[i]);
    }

    Serial.println("BleMessageQueue::push_queue->Added Message");
    Serial.print(queue[size].command);
    Serial.print("\t");
    Serial.println(queue[size].expecting);
    for (int i = 0; i < (queue[size].expecting); i++)
    {
        Serial.print(queue[size].data[i]);
        Serial.print("\t");
    }
    Serial.println();
}

int queue_size()
{
    return size;
}
#endif