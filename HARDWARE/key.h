#ifndef KEY_H
#define KEY_H

#include "gpio.h"
#include "main.h"


typedef struct{
uint16_t _1;
uint16_t _2;
uint16_t _3;
uint16_t _4;
uint16_t _5;
}Key; 

typedef enum
{
  KEY_UP   = 0,
  KEY_DOWN = 1,
}KEYState;
#define KEY_DOWN_LEVEL	0				//°´ÏÂÎª0

KEYState read_key_state_gpio(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
void read_key_state(void);
void motor_key_test(void);


#endif

