#include "key.h"
#include "DJI_Motor.h"


Key key;

KEYState read_key_state_gpio(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{
	static uint8_t count=0;
		if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==KEY_DOWN_LEVEL)
		{
			if(count<200)
			count++;

			if(count>4)
			{
				return KEY_DOWN;
			}
		}else 
		{
			count=0;
			return KEY_UP;
		}
		return KEY_DOWN;
}


void read_key_state(void)
{
	key._1 = read_key_state_gpio(GPIOE,GPIO_PIN_0);
	key._2 = read_key_state_gpio(GPIOE,GPIO_PIN_1);
	key._3 = read_key_state_gpio(GPIOE,GPIO_PIN_2);
	key._4 = read_key_state_gpio(GPIOE,GPIO_PIN_3);
	key._5 = read_key_state_gpio(GPIOE,GPIO_PIN_4);
}


void motor_key_test(void)
{
//	if(key._1==KEY_DOWN)
//	{
//		ZL_Motor[0].target_speed_rpm=100;
//		ZL_Motor[1].target_speed_rpm=100;
//	}
//	if(key._2==KEY_DOWN)
//	{
//		ZL_Motor[0].target_speed_rpm++;
//	}
//		if(key._3==KEY_DOWN)
//	{
//		ZL_Motor[0].target_speed_rpm--;
//	}
//	if(key._4==KEY_DOWN)
//	{
//	ZL_Motor[0].target_speed_rpm=0;
//	ZL_Motor[1].target_speed_rpm=0;
//	}

}

