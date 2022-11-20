#ifndef BATTERY_H
#define BATTERY_H
#include "pc_data_struct.h"
#include "non_time_task.h"


extern const char* cmd_readBattery;
extern uint8_t uart3_rx_data[50];
extern uint8_t Battery_req_flag;

void check_Battery_remain_send(void);
void decode_Battery_info(uint8_t *buff,uint8_t len);



#endif


