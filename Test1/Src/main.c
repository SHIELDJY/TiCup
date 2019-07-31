/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* 头文件 ------------------------------------------------------------------*/
#include "main.h"

/* 私有头文件 ----------------------------------------------------------*/
/* 私有类 -----------------------------------------------------------*/
/* 私有定义 ------------------------------------------------------------*/
/* 私有宏 -------------------------------------------------------------*/

/* 私有变量 ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef UartHandle;

/* 私有函数原型 -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void UART_Init(void);
static void DAC_Ch2_TriangleConfig(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{  
	uint32_t temp_dac = 0;

  /* MCU 配置--------------------------------------------------------*/

  /* 重置所有外设, 初始化 Flash 与系统时钟. */
  HAL_Init();
	HAL_DAC_DeInit(&hdac);

  /* 配置系统时钟 */
  SystemClock_Config();


  /* 初始化外设 */
  MX_GPIO_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
	UART_Init();
	
	DAC_Ch2_TriangleConfig();

  /* Infinite loop */
  while (1)
  {
		temp_dac = HAL_DAC_GetValue(&hdac, DAC_CHANNEL_2);
  }
}

/**
  * @brief 系统时钟配置
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC初始化函数
  * @param 无
  * @retval 无
  */
static void MX_DAC_Init(void)
{

  /** 定义DAC通道初始化类 sConfig
	*/
  DAC_ChannelConfTypeDef sConfig = {0};
  /** DAC 初始化
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC 通道 OUT2 配置
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE; //无外部源触发转换
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE; //使能输出缓冲
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK) //等待配置成功
  {
    Error_Handler();
  }

}
/**
  * @brief UART 初始化函数
  * @param None
  * @retval None
  */
static void UART_Init(void)
{
	uint32_t temp_time;
	UartHandle.Instance          = USARTx;
  
  UartHandle.Init.BaudRate     = 9600;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_ODD;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
  
  /* Output a message on Hyperterminal using printf function */
//  printf("\n\r UART Printf Example: retarget the C library printf function to the UART\n\r");
	HAL_UART_Transmit(&UartHandle,"Hello World",12,temp_time);
}
/**
  * @brief TIM6 初始化函数
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
	htim6.Init.ClockDivision = 0; 
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0x7ff;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
	HAL_TIM_Base_Start(&htim6);
}

/**
  * @brief GPIO 初始化函数
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO 端口时钟使能 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*配置 GPIO 引脚 : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
	
  /*配置 GPIO 引脚 : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USARTx_TX_PIN|USARTx_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}


/**
  * @brief  DAC 通道2 三角波配置
  * @param  None
  * @retval None
  */
static void DAC_Ch2_TriangleConfig(void)
{
  /**-3- 配置 DAC 通道2 产生三角波 
	*/
  if(HAL_DACEx_TriangleWaveGenerate(&hdac, DAC_CHANNEL_2, DAC_TRIANGLEAMPLITUDE_1023) != HAL_OK)
  {
    /* Triangle wave generation Error */
    Error_Handler();
  }
  /**-4- 使能 DAC 通道2 
	*/  
  if(HAL_DAC_Start(&hdac, DAC_CHANNEL_2) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
  /**-5- 设置 DAC 通道2 DHR12RD 寄存器
	*/
  if(HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0x100) != HAL_OK)
  {
    /* Setting value Error */
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
