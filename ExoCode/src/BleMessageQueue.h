#ifndef BLEMESSAGEQUEUE_H
#define BLEMESSAGEQUEUE_H

#include "BleMessage.h"

BleMessage pop_queue();
void push_queue(BleMessage* msg);
int queue_size();
int check_queue_for(BleMessage msg);

#endif