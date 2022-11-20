#include "sbus.h"


uint16_t sbus_CH[18];
uint8_t sbus_connect=0;


struct Command_Sbus Command2PC;
struct jaka_tcp_move jaka_tcp_move2PC;

void sbus_dcode(uint16_t *CH ,uint8_t *buf)
{
	CH[ 0] = ((int16_t)buf[ 2-1] >> 0 | ((int16_t)buf[ 3-1] << 8 )) & 0x07FF;
	CH[ 1] = ((int16_t)buf[ 3-1] >> 3 | ((int16_t)buf[ 4-1] << 5 )) & 0x07FF;
	CH[ 2] = ((int16_t)buf[ 4-1] >> 6 | ((int16_t)buf[ 5-1] << 2 ) | (int16_t)buf[ 6-1] << 10 ) & 0x07FF;
	CH[ 3] = ((int16_t)buf[ 6-1] >> 1 | ((int16_t)buf[ 7-1] << 7 )) & 0x07FF;
	CH[ 4] = ((int16_t)buf[ 7-1] >> 4 | ((int16_t)buf[ 8-1] << 4 )) & 0x07FF;
	CH[ 5] = ((int16_t)buf[ 8-1] >> 7 | ((int16_t)buf[ 9-1] << 1 )| (int16_t)buf[10-1] <<  9 ) & 0x07FF;
	CH[ 6] = ((int16_t)buf[10-1] >> 2 | ((int16_t)buf[11-1] << 6 ))& 0x07FF;
	CH[ 7] = ((int16_t)buf[11-1] >> 5 | ((int16_t)buf[12-1] << 3 ))& 0x07FF;
	
	CH[ 8] = ((int16_t)buf[13-1] << 0 | ((int16_t)buf[14-1] << 8 )) & 0x07FF;
	CH[ 9] = ((int16_t)buf[14-1] >> 3 | ((int16_t)buf[15-1] << 5 )) & 0x07FF;
	CH[10] = ((int16_t)buf[15-1] >> 6 | ((int16_t)buf[16-1] << 2 )) & 0x07FF;
	CH[11] = ((int16_t)buf[17-1] >> 1 | ((int16_t)buf[18-1] << 7 ))	& 0x07FF;
	CH[12] = ((int16_t)buf[18-1] >> 4 | ((int16_t)buf[19-1] << 4 )) & 0x07FF;
	CH[13] = ((int16_t)buf[19-1] >> 7 | ((int16_t)buf[20-1] << 1 ) | (int16_t)buf[21-1] <<  9 ) & 0x07FF;
	CH[14] = ((int16_t)buf[21-1] >> 2 | ((int16_t)buf[22-1] << 6 )) & 0x07FF;
	CH[15] = ((int16_t)buf[22-1] >> 5 | ((int16_t)buf[23-1] << 3 ))& 0x07FF;

	if(CH[2]<100)
	{Command2PC.mode=MODE_STOP;
	sbus_connect = DIS_CONNECTED;
	//如果是这样的话，遥控器关机
	}
	else
	{
		sbus_connect = CONNECTED;
		
		if(CH[4]>190&&CH[4]<210)
		{Command2PC.mode=MODE_STOP;
		}else if(CH[4]>990&&CH[4]<1100)
		{Command2PC.mode=MODE_NORMAL;}
		else if(CH[4]>1700&&CH[4]<1900)
		{Command2PC.mode=MODE_BACK_ZERO;}
		
		Command2PC.vw=(int32_t)CH[3]-1000;//正负800,左操纵杆左右，左边为正
		Command2PC.vx=(int32_t)CH[0]-1000;//正负800,右操纵杆左右，左边为正
		Command2PC.vy=(int32_t)CH[1]-1000;//正负800,右操纵杆上下，右边为正
		
	}//遥控器正常连接
}


void dji_dbus_decode(uint16_t *CH ,uint8_t *buf)
{
	CH[ 0] = ((int16_t)buf[0]  | ((int16_t)buf[1] << 8 )) & 0x07FF;
	CH[ 1] = (((int16_t)buf[1] >> 3) | ((int16_t)buf[2] << 5 )) & 0x07FF;
	CH[ 2] = (((int16_t)buf[2] >> 6) | ((int16_t)buf[3] << 2 ) | ((int16_t)buf[4] << 10) ) & 0x07FF;
	CH[ 3] = (((int16_t)buf[4] >> 1) | ((int16_t)buf[ 5] << 7 )) & 0x07FF;
	CH[ 4] = ((int16_t)buf[ 5] >> 4) & 0x03;
	CH[ 5] = ((int16_t)buf[ 5] >> 6) & 0x03;
	CH[6]=((int16_t)buf[16]) | ((int16_t)buf[ 17]<<8);
		switch(CH[4])
		{case 3:
			Command2PC.mode=MODE_STOP;
			break;
			case 2:
			Command2PC.mode=MODE_NORMAL;
			break;
			case 1:
			Command2PC.mode=MODE_BACK_ZERO;
			break;
		}
		
		
		Command2PC.vw=(int32_t)CH[2]-1024;//正负800,左操纵杆左右，左边为正
		Command2PC.vx=(int32_t)CH[0]-1024;//正负800,右操纵杆左右，左边为正
		Command2PC.vy=(int32_t)CH[1]-1024;//正负800,右操纵杆上下，右边为正
		
		
		
	
	
	
}


