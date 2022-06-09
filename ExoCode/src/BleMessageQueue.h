#ifndef BLEMESSAGEQUEUE_H
#define BLEMESSAGEQUEUE_H

#if defined(ARDUINO_ARDUINO_NANO33BLE)

#include "BleMessage.h"

BleMessage pop_queue();
void push_queue(BleMessage* msg);
int queue_size();

#endif
#endif