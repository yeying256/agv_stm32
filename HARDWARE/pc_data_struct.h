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
//#define MODE_STOP 0		//ֹͣģʽ��
//#define MODE_NORMAL 1 //����ģʽ
//#define MODE_BACK_ZERO 2;//����ģʽ
enum{
MODE_STOP = 0,
MODE_NORMAL,
MODE_BACK_ZERO
};
struct Command_Sbus{
	uint8_t mode;

	//����λ
	//0:MODE_STOPֹͣģʽ��
	//1:MODE_NORMAL����ģʽ
	//2:MODE_BACK_ZERO
	int32_t vx;
	int32_t vy;
	int32_t vw;
};//13���ֽ�

#define CMD_jaka_tcp_move (0x0002)

struct jaka_tcp_move{
	uint8_t mode;
	//����λ
	//0:MODE_STOPֹͣģʽ��
	//1:MODE_NORMAL����ģʽ
	//2:MODE_BACK_ZERO
	int32_t vx;
	int32_t vy;
	int32_t vz;
	int32_t wx;
	int32_t wy;
};//13���ֽ�


struct state_non_realtime{
	
	uint8_t battery_reamin;
};//







#pragma pack()


void send2PC(uint16_t CMD,uint8_t *data,uint8_t len);



#endif


