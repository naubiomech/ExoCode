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

namespace UART_msg_t_utils
{
    static void print_msg(UART_msg_t msg)
    {
        Serial.println("UART_command_utils::print_msg->Msg: ");
        Serial.print(msg.command); Serial.print("\t");
        Serial.print(msg.joint_id); Serial.print("\t");
        Serial.print(msg.len); Serial.println();
        for (int i=0; i<msg.len; i++)
        {
           Serial.print(msg.data[i]); Serial.print(", ");
        }
        Serial.println();
    }
};


#endif