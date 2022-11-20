#include "non_time_task.h"


uint8_t recv_flag_485 = 0;

struct state_non_realtime state_NoRT;




void decode_485_info(uint8_t rx_num)
{
	if(uart3_rx_data[0]==':')
	{
		decode_Battery_info(uart3_rx_data,rx_num);
	}
}

void non_time_task(void)
{
		static uint32_t time7_flag_last;
//			sbus_dcode(sbus_CH ,sbus_data);
	if(usart2_rx_flag==1)
	{dji_dbus_decode(sbus_CH ,sbus_data);
		
	usart2_rx_flag=0;
	}
	if(time7_flag_last!=time7_flag)
	{
		send2PC(CMD_SBUS,(uint8_t *)(&Command2PC),sizeof(Command2PC));
		EEhand_control();
	}
	time7_flag_last=time7_flag;
	if(recv_flag_485==1)
	{
		decode_485_info(uart3_rx_num);
		recv_flag_485=0;
		
		//HAL_GPIO_WritePin(GPIOH,GPIO_PIN_8,GPIO_PIN_RESET);//关闭485使能
	}
	
	if(Battery_req_flag==1)//检查电池状态
	{
		check_Battery_remain_send();
		Battery_req_flag=0;
	
	}
	
	
}

	uint8_t hand_buf[8] = {0x01,0x06,0x00,0x0b,0x00,0x00,0xF8,0x08};//抓取物体（正向）
	uint8_t hand_buf2[8] = {0x01,0x06,0x00,0x0b,0x00,0x01,0x39,0xC8};//释放物体（反向）
	
	uint8_t hand_run_buf[8] = {0x01,0x05,0x00,0x07,0xff,0x00,0x3d,0xfb};

void EEhand_control()
{
	static uint8_t last_ch5;
	static uint8_t cmd_ON=0;

	
	
	if(cmd_ON == 1)
	{
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_8,GPIO_PIN_SET);
		HAL_UART_Transmit(&huart3,hand_run_buf,8,16);
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_8,GPIO_PIN_RESET);
		cmd_ON = 0;
	}
	
	
	
	if(sbus_CH[5]==1 && last_ch5!=sbus_CH[5])
	{
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_8,GPIO_PIN_SET);//开启485使能
	
		HAL_UART_Transmit(&huart3,hand_buf,8,16);//抓取物体
		
		//HAL_UART_Transmit(&huart3,run_buf,8,16);//485是uart3
		
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_8,GPIO_PIN_RESET);//关闭485使能
		cmd_ON = 1;
	}
	
	if(sbus_CH[5]==2 && last_ch5!=sbus_CH[5])
	{
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_8,GPIO_PIN_SET);//开启485使能
	
		HAL_UART_Transmit(&huart3,hand_buf2,8,16);//485是uart3
		
		//HAL_UART_Transmit(&huart3,run_buf,8,16);//485是uart3
		
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_8,GPIO_PIN_RESET);//关闭485使能
		cmd_ON = 1;
	}

	
	
	
	
	last_ch5=sbus_CH[5];

}


//ask码 转16进制
unsigned char CharToHex(unsigned char bChar)
{
		if((bChar >= 0x30)&&(bChar <= 0x39))
		{
			bChar -= 0x30;
		}
		else if((bChar>= 0x41)&&(bChar<= 0x46)) // Capital
		{
			bChar -= 0x37;
		}
		else if((bChar>= 0x61)&&(bChar<= 0x66)) //littlecase
		{
			bChar -= 0x57;
		}
		else
		{
			bChar = 0xff;
		}
	return bChar;
}

unsigned char HexToChar(unsigned char bHex)
{
	if(bHex<=9)
	{
		bHex += 0x30;
	}
	else if((bHex>=10)&&(bHex<=15))//Capital
	{
		bHex += 0x37;
	}
	else
	{
		bHex = 0xff;
	}
	return bHex;
}




