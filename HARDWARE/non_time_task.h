#ifndef NON_TIME_TASK_H
#define NON_TIME_TASK_H

#include "stdio.h"
#include "pc_data_struct.h"
#include "sbus.h"
#include "battery.h"

extern uint32_t time7_flag;
extern uint8_t usart2_rx_flag;
extern uint8_t recv_flag_485;
extern uint8_t uart3_rx_num;
extern struct state_non_realtime state_NoRT;

extern uint8_t uart3_rx_data[50];

void non_time_task(void);
unsigned char HexToChar(unsigned char bChar);
unsigned char CharToHex(unsigned char bHex);
void EEhand_control();
























#endif

