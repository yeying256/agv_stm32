#ifndef SBUS_DECODE_H
#define SBUS_DECODE_H

#include "stdio.h"

#include "pc_data_struct.h"


void sbus_dcode(uint16_t *CH ,uint8_t *buf);
void dji_dbus_decode(uint16_t *CH ,uint8_t *buf);

#define DIS_CONNECTED 0
#define CONNECTED 1
#define ERR 2
#define DATA_IN 3


#define sbus_data_len 18



extern uint16_t sbus_CH[18];
extern uint8_t sbus_connect;
extern uint8_t sbus_data[sbus_data_len];

extern struct Command_Sbus Command2PC;









#endif

