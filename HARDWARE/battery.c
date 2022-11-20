#include "battery.h"

const char* cmd_readBattery = ":001300000E06~";
uint8_t Battery_req_flag=0;




void check_Battery_remain_send(void)
{
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_8,GPIO_PIN_SET);//����485ʹ��
	
		HAL_UART_Transmit(&huart3,(uint8_t*)cmd_readBattery,strlen(cmd_readBattery),16);//485��uart3
		
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_8,GPIO_PIN_RESET);//�ر�485ʹ��
}

void decode_Battery_info(uint8_t *buff,uint8_t len)
{
	const uint8_t tmp_cmp[] =":019352001A";
	if(strncmp((const char*)buff,(const char*)tmp_cmp,11) == 0)//�鿴��ص���
	{
		state_NoRT.battery_reamin = (uint8_t)((CharToHex(buff[11])<<4)|(CharToHex(buff[12])))&0x00ff;
		
	}
}
