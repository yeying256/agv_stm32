#ifndef PC_DATA_STRUCT

#define PC_DATA_STRUCT

#include "stddef.h"
#include "stdint.h"

#include "stdio.h"
#include "usart.h"
#include "string.h"

extern uint32_t CMD2PC;

//#define SOF 0xA5


void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);


#pragma pack(1) 
struct Frame_header{
	uint8_t SOF;
	uint16_t data_length;
	uint8_t CRC8code;
};

#define CMD_SBUS (0x0001)
//#define MODE_STOP 0		//停止模式，
//#define MODE_NORMAL 1 //正常模式
//#define MODE_BACK_ZERO 2;//回零模式
enum{
MODE_STOP = 0,
MODE_NORMAL,
MODE_BACK_ZERO
};
struct Command_Sbus{
	uint8_t mode;

	//低四位
	//0:MODE_STOP停止模式，
	//1:MODE_NORMAL正常模式
	//2:MODE_BACK_ZERO
	int32_t vx;
	int32_t vy;
	int32_t vw;
};//13个字节

#define CMD_jaka_tcp_move (0x0002)

struct jaka_tcp_move{
	uint8_t mode;
	//低四位
	//0:MODE_STOP停止模式，
	//1:MODE_NORMAL正常模式
	//2:MODE_BACK_ZERO
	int32_t vx;
	int32_t vy;
	int32_t vz;
	int32_t wx;
	int32_t wy;
};//13个字节


struct state_non_realtime{
	
	uint8_t battery_reamin;
};//







#pragma pack()


void send2PC(uint16_t CMD,uint8_t *data,uint8_t len);



#endif


