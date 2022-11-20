/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
	
	
	
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "DJI_Motor.h"
#include "key.h"

#include "string.h"

#include "sbus.h"





/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */






/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int32_t Motor_Position_Target[2]={0};
extern uint32_t a;


uint8_t uart4_rx_data[2];

uint8_t uart3_rx_data;
#define sbus_data_len 25
uint8_t sbus_data[sbus_data_len];

int32_t uart3_test_count=0;
uint8_t PD6;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

//	PID_struct_init(&Motor_Position_Loop[0],POSITION_PID,9100,9100,0.5f,0,0);
//	PID_struct_init(&Motor_Position_Loop[1],POSITION_PID,9100,9100,0.5f,0,0);
//	PID_struct_init(&Motor_Speed_Loop[0],POSITION_PID,20000,20000,7.5f,0.49f,0);
//	PID_struct_init(&Motor_Speed_Loop[1],POSITION_PID,20000,20000,7.5f,0.49f,0);

	
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
		



  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	CAN_Filter_Init();
	
	HAL_TIM_Base_Init(&htim6);
	HAL_TIM_Base_Start_IT(&htim6);
	
	HAL_TIM_Base_Init(&htim7);
	HAL_TIM_Base_Start_IT(&htim7);
	
	//从串口4的buff区拷贝数据到全局变量中的函数
	HAL_UART_Receive_IT(&huart4,uart4_rx_data,2);
	
	HAL_UART_Receive_IT(&huart3,&uart3_rx_data,1);//485通信使能
	
//	HAL_UART_Receive_IT(&huart6,sbus_data,25);//sbus通信使能
	HAL_UART_Receive_IT(&huart2,sbus_data,sbus_data_len);//sbus通信使能
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_8,GPIO_PIN_RESET);//接收模式，发送模式是GPIO_PIN_SET
	
	
		ZL_Motor_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		PID_calc(&Motor_Position_Loop[0],DJI_Motor[0].total_angle,Motor_Position_Target[0]);
//		PID_calc(&Motor_Position_Loop[1],DJI_Motor[1].total_angle,Motor_Position_Target[1]);
//		PID_calc(&Motor_Speed_Loop[0],DJI_Motor[0].speed_rpm,Motor_Position_Loop[0].pos_out);
//		PID_calc(&Motor_Speed_Loop[1],DJI_Motor[1].speed_rpm,Motor_Position_Loop[1].pos_out);

//		Set_Four_Current(1,22,3,4,0x100);
		

		
//		HAL_Delay(1);
//		read_key_state();
//		a++;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		

		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uart3_test_count++;
	
	if(huart->Instance == UART4)
	{
		
	}
	;
		if(huart->Instance == USART3)
	{
		HAL_UART_Receive_IT(&huart3,&uart3_rx_data,1);
//		HAL_UART_Transmit(,);
		uint8_t tmp[10];
		memcpy(tmp,"fuck",50);
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_8,GPIO_PIN_SET);
//		HAL_UART_Transmit(&huart3,tmp,strlen((char *)tmp),1000);
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_8,GPIO_PIN_RESET);
	}
	
		if(huart->Instance == USART2)
	{
	HAL_UART_Receive_IT(&huart2,sbus_data,sbus_data_len);
		
		
		
		sbus_dcode(sbus_CH ,sbus_data);
//		send2PC(CMD_SBUS,(uint8_t *)(&Command2PC),sizeof(Command2PC));
		
	}
}





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
