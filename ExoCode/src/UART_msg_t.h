#ifndef UARTMSG_H
#define UARTMSG_H

#define UART_MSG_T_MAX_DATA_LEN 64
#include "Arduino.h"

typedef struct
{
  uint8_t command;
  uint8_t joint_id;
  float data[UART_MSG_T_MAX_DATA_LEN];
  uint8_t len;
} UART_msg_t;


#endif