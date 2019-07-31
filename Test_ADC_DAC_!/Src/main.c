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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"
#include "arm_const_structs.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define pi 3.14159265
#define NPT  1024  //1024点FFT
#define Fs 200000  //采样频率 200kHz 频率分辨率 195Hz
#define PI2 6.28318530717959
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

UART_HandleTypeDef huart2;

TIM_HandleTypeDef htim2;

uint16_t hadc1_val;
uint16_t hadc1_val_last;
uint16_t test_freq;
uint32_t sinewave_val[64];
float32_t fftout[1024];
float32_t fftout_amp[1024];
float32_t  testInput_f32[NPT];
float32_t  testOutput_f32[NPT];
float32_t  testOutput[NPT];

/* Private function prototypes -----------------------------------------------*/
/* Initial prototypes ----------*/
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Config(void);
static void MX_DMA_Init(void);
/* User prototypes ----------*/
uint16_t ADC_GetValue(ADC_HandleTypeDef *hadc);
void Sine_CalValue(uint32_t* pdata, uint16_t datasize);

/* Private user code ---------------------------------------------------------*/
void arm_rfft_fast_f32_app(void) 
{ 
  uint16_t i; 
  arm_rfft_fast_instance_f32 S; 

  /* Length of sequence */ 
  uint16_t fftSize = NPT;  
  /* fourier trans */ 
	uint8_t ifftFlag = 0;  

  /* initialize S struct */ 
   arm_rfft_fast_init_f32(&S, fftSize); 

    /* 按照实部，虚部，实部，虚部..... 的顺序存储数据 */ 
  for(i=0; i<1024; i++) 
  { 
    /*3种频率 50Hz 2500Hz 2550Hz */ 
    testInput_f32[i] = 1000*arm_sin_f32(PI2*i*50.0/Fs) + 
               2000*arm_sin_f32(PI2*i*2500.0/Fs)  + 
               3000*arm_sin_f32(PI2*i*2550.0/Fs); 
  } 

  /* 1024点实序列快速FFT */  
  arm_rfft_fast_f32(&S, testInput_f32, testOutput_f32, ifftFlag); 

  /* 为了方便跟函数arm_cfft_f32计算的结果做对比，这里求解了1024组模值，实际函数arm_rfft_fast_f32 
     只求解出了512组   
  */  
   arm_cmplx_mag_f32(testOutput_f32, testOutput, fftSize); 
			for(i=0;i<512;i++)
				VOSC_SendOne_uint16(&huart2, (uint16_t)testOutput[i]);

}

/**
  * @brief  The application entry point.
  * @retval int
  */
	uint16_t temp;
int main(void)
{
	uint16_t i;
	arm_rfft_fast_instance_f32 S; 
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	
	/* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Initialize all configured peripherals */
	Sine_CalValue(sinewave_val,64);
  MX_GPIO_Init();
  MX_ADC1_Init();
	MX_DMA_Init();
  MX_DAC_Init();
	MX_TIM2_Config();
  MX_USART2_UART_Init();
	
	

  /* 状态指示灯亮（LD2绿灯） */
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);

//	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);//设定DAC通道1（PA4的值）[0-4095]

  /* Output a message on Hyperterminal using printf function */
  printf("Hello World!\n\r");
	printf("hadc1_val=%d\n",hadc1_val);
	
//	arm_rfft_fast_f32_app();
	  /* initialize S struct */ 
  while (1)
  {

		/*10kHz Sinwave Amp=825mV DCBias=1.65V */ 
//		testInput_f32[i] = 1000*arm_sin_f32(PI2*i*2000.0/Fs) + 1500;
    
//		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, sinewave_val[i]);//设定DAC通道1（PA4的值）[0-4095]
//    temp = ADC_GetValue(&hadc1);
//		i++;
////		VOSC_SendOne_uint16(&huart2, sinewave_val[i]);
//    if(i==1000) i=0;
////    {
////			arm_rfft_fast_init_f32(&S, 1024); 
////      arm_rfft_fast_f32(&S, sinewave_val, fftout, 0); 
////      arm_cmplx_mag_f32(fftout, fftout_amp, 1024); 
////			for(i=0;i<1024;i++)
////			{
////				if(fftout_amp[i]>50000)
////					test_freq = 195*i;
////					VOSC_SendOne_uint16(&huart2, (uint16_t)temp);
////				VOSC_SendTwo_uint16(&huart2, (uint16_t)fftout_amp[i],test_freq);
////			}
////			i=0;
//			HAL_Delay(10);
//    }
//		printf("hadc1_val=%d\n",hadc1_val);
  }
}
/**
  * @brief Calculate Values of a Sinewave
	& @param * @pdata pointer of the data @datasize size of the pointer
  * @retval None
  */
void Sine_CalValue(uint32_t* pdata, uint16_t datasize)
{
	for(int i=0; i<datasize; i++)
	{
		pdata[i] = 1000*arm_sin_f32(PI2*i/64.0) + 1500;
	}
}
/**
  * @brief GetValue From adc channel
  * @retval None
  */
uint16_t ADC_GetValue(ADC_HandleTypeDef *hadc)
{
  uint16_t adc_val;
	HAL_ADC_PollForConversion(hadc, 10);
	if ((HAL_ADC_GetState(hadc) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG)
	{
		HAL_ADC_Start(hadc);
		adc_val = HAL_ADC_GetValue(hadc);
	}

  return adc_val;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
	if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    /* Start Conversation Error */
    Error_Handler();
  }
  /*  Before starting a new conversion, you need to check the current state of
       the peripheral; if it's busy you need to wait for the end of current
       conversion before starting a new one.
       For simplicity reasons, this example is just waiting till the end of the
       conversion, but application may perform other tasks while conversion
       operation is ongoing. */
  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
  {
    /* End Of Conversion flag not set on time */
    Error_Handler();
  }

  /* Check if the continuous conversion of regular channel is finished */
  if ((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG)
  {
    /*##-5- Get the converted value of regular channel  ########################*/
    hadc1_val = HAL_ADC_GetValue(&hadc1);
  }

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{
  DAC_ChannelConfTypeDef sConfig = {0};

  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC通道1（PA4）打开 
  */
//	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t *)sinewave_val,64,DAC_ALIGN_12B_R);
}

/**
  * @brief  DAC 通道1 三角波配置
  * @param  None
  * @retval None
  */
static void DAC_Ch1_TriangleConfig(void)
{
  /**-3- 配置 DAC 通道1 产生三角波 
	*/
  if(HAL_DACEx_TriangleWaveGenerate(&hdac, DAC_CHANNEL_1, DAC_TRIANGLEAMPLITUDE_4095) != HAL_OK)
  {
    /* Triangle wave generation Error */
    Error_Handler();
  }
  /**-4- 使能 DAC 通道1 
	*/  
  if(HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
  /**-5- 设置 DAC 通道1 DHR12RD 寄存器
	*/
  if(HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x100) != HAL_OK)
  {
    /* Setting value Error */
    Error_Handler();
  }
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
	DMA_HandleTypeDef hdma;
  
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma.Init.Channel = DMA_CHANNEL_7;
  hdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma.Init.MemInc = DMA_MINC_ENABLE;
  hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma.Init.MemDataAlignment = DMA_PDATAALIGN_WORD;
  hdma.Init.Mode = DMA_CIRCULAR;
  hdma.Init.Priority = DMA_PRIORITY_MEDIUM;
  hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma.Init.PeriphBurst = DMA_PBURST_SINGLE;

  hdma.Instance = DMA1_Stream5;

  HAL_DMA_Init(&hdma);

  HAL_DMA_Start(&hdma,(uint32_t)&sinewave_val,(uint32_t)&(DAC->DHR12R1),64);

  // /* DMA interrupt init */
  // /* DMA1_Stream5_IRQn interrupt configuration */
  // HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}
/**
  * @brief  TIM2 Configuration
  * @note   TIM2 configuration is based on APB1 frequency
  * @note   TIM2 Update event occurs each TIM2CLK/256
  * @param  None
  * @retval None
  */
static void MX_TIM2_Config(void)
{
	HAL_TIM_Base_MspInit(&htim2);
	
	__HAL_RCC_TIM2_CLK_ENABLE();
	
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	HAL_TIM_Base_Start(&htim2);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF); 

  return ch;
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
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
