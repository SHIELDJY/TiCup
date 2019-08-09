/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#define ADC_CHANNEL_SIZE       4
#define Rs 2 //Rs=2kohm
#define NbrPTs 512
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;

uint16_t hadc1_val;
uint16_t hadc1_val_last;
uint16_t test_freq;
uint32_t sinewave_val[64];
uint16_t tempval1[NbrPTs];
uint16_t tempval2[NbrPTs];
uint16_t tempval3[NbrPTs];
uint32_t ADCConvertedValue[ADC_CHANNEL_SIZE];
__IO uint16_t uhADCxConvertedValue[5] = {0};
float ADC_Value[ADC_CHANNEL_SIZE];  //用来保存经过转换得到的电压值
int temp;
float f;
float temp_max[4]={0};
float temp_min[4]={3.3};
float temp_rms,temp_bias[4];

/* Private function prototypes -----------------------------------------------*/
/* Initial prototypes ----------*/

/* User prototypes ----------*/
uint16_t ADC_GetValue(ADC_HandleTypeDef *hadc);
void Sine_CalValue(uint32_t* pdata, uint16_t datasize);
void Result_CalValue(void);
void Signal_Process(void);
void ResultDisplay(void);

/* Private user code ---------------------------------------------------------*/


/**
  * @brief  The application entry point.
  * @retval int
  */
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
//  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Initialize all configured peripherals */
	Sine_CalValue(sinewave_val,64);
	MX_USART1_UART_Init();
	MX_GPIO_Init();
	LCD_Clear();
	LCD_Initialize();
	LCD_Initialize();
	HAL_Delay(1000);
	LCD_PrintChar("Welcome",0,24);
	MX_DMA_Init();
	MX_ADC1_Init();
  MX_DAC_Init();
	MX_TIM2_Config();
	MX_TIM3_Config();
	MX_TIM4_Config();
  /* 状态指示灯亮（LD2绿灯） */
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&uhADCxConvertedValue, 3);
  /* Output a message on Hyperterminal using printf function */
//  printf("Hello World!\n\r");
//	printf("hadc1_val=%d\n",hadc1_val);
	HAL_Delay(2000);
	LCD_PlotInit();
  while (1)
  {
		Signal_Process();
		Result_CalValue();
		ResultDisplay();
		HAL_Delay(500);
		/**
		 * Virtual Oscilloscope Output
		 */
	 for(int i=0;i<NbrPTs;i++)
	 {
		VOSC_SendTwo_uint16(&huart1, 1000*tempval3[i]/4095.0*3.38,1000*temp_bias[3]);
		HAL_Delay(5);
	 }
		// /**
		//  * Read Values from DMA register
		//  */
		// for(int j=0;j<NbrPTs;j++)
		// {
		// 	tempval1[j] = uhADCxConvertedValue[0];
		// 	tempval2[j] = uhADCxConvertedValue[2];
		// 	tempval3[j] = uhADCxConvertedValue[4];
		// 	i = 0;
		// 	while(i!=100) i++;
		// }
		// /**
		//  * Peak Value Calculation
		//  */
		// for(int j = 0;j <NbrPTs; j++)
		// {
		// 	ADC_Value[1] = tempval1[j]/4095.0*3.3;
		// 	ADC_Value[2] = tempval2[j]/4095.0*3.3;
		// 	ADC_Value[3] = tempval3[j]/4095.0*3.3;
		// 	if(ADC_Value[1]>temp_max[1])temp_max[1]=ADC_Value[1];
		// 	else if(ADC_Value[1]<temp_min[1]) temp_min[1]=ADC_Value[1];
		// 	if(ADC_Value[2]>temp_max[2])temp_max[2]=ADC_Value[2];
		// 	else if(ADC_Value[2]<temp_min[2]) temp_min[2]=ADC_Value[2];
		// 	if(ADC_Value[3]>temp_max[3])temp_max[3]=ADC_Value[3];
		// 	else if(ADC_Value[3]<temp_min[3]) temp_min[3]=ADC_Value[3];
		// }
		// /**
		//  * DC Bias Calculation
		//  */
		// for(int j = 1; j < 4; j++)
		// {
		// 	temp_bias[j] = (temp_max[j]+temp_min[j])/2.0;
		// }
		// for(int j = 1; j < 4; j++){temp_max[j] = 0;temp_min[j] = 3.3;}	

	}
//		i++;
//		f = 4.0/(float)i;
//		LCD_PrintUint16_t(i,0,50);
//		LCD_PrintFloat(f,0,70);
//		LCD_PlotPoint(i);
//		HAL_Delay(10);
//		LCD_PrintChar("Hello world",0,20);
//		printf("Hello World!\n\r");
//		i++;
//		VOSC_SendOne_uint16(&huart1, sinewave_val[i]);
//    if(i==1000) i=0;
//    {
//			arm_rfft_fast_init_f32(&S, 1024); 
//      arm_rfft_fast_f32(&S, sinewave_val, fftout, 0); 
//      arm_cmplx_mag_f32(fftout, fftout_amp, 1024); 
//			for(i=0;i<1024;i++)
//			{
//				if(fftout_amp[i]>50000)
//					test_freq = 195*i;
//				VOSC_SendTwo_uint16(&huart1, (uint16_t)fftout_amp[i],test_freq);
//			}
//			i=0;
//    }
}
/**
  * @brief Calculate Values of a desired results
	& @param * none
  * @retval None
  */
float Ui,Us,Uo,Uop;
float Ri, Ro,Ro_old, Au;
float Uopp_pos, Uo_bias,Uo_rms;
float Uopp_neg=3.3;
float Uipp_pos,Uipp_neg, Ui_bias,Ui_rms;
float Uop_pos,Uop_neg, Uop_bias,Uop_rms;
//float 
void Result_CalValue(void)
{
	Us = 0.01414;
	Ri = Ui_rms*Rs/(Us-Ui_rms);
	Ro_old = Ro;
	Ro = (Uo_bias/Uop_bias-1)*80;
	if (Ro < 0 ) Ro = Ro_old;
	
//	Au = Uo_rms/Ui_rms;
	Au = 0;
}
/**
  * @brief Dispaly Results
	& @param * none
  * @retval None
  */
void ResultDisplay(void)
{
	LCD_PrintChar("Ri =",0,70);
	HAL_Delay(10);
	LCD_PrintFloat(Ri,50,70);
	HAL_Delay(10);
	LCD_PrintChar("kOhm",120,70);
	HAL_Delay(10);
	LCD_PrintChar("Ro =",0,90);
	HAL_Delay(10);
	LCD_PrintFloat(Ro,50,90);
	HAL_Delay(10);
	LCD_PrintChar("kOhm",120,90);
	HAL_Delay(10);
	LCD_PrintChar("Au =",0,110);
	HAL_Delay(10);
	LCD_PrintFloat(Au,50,110);
	HAL_Delay(10);
	LCD_PrintChar("V/V",120,110);
	HAL_Delay(10);
}
/**
  * @brief Process input signals
	& @param None
  * @retval None
  */
void Signal_Process(void)
{
	uint16_t ADC_Value_Raw;
	uint16_t temp_adc;
	/**
	 * Read Ui and Uo from adc without load resistance
	 */
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET); //with load
	HAL_Delay(500);
	for(int j=0;j<NbrPTs;j++)
	{
		for(int i = 0; i<5; i++)
		{
			ADC_Value_Raw += uhADCxConvertedValue[4];
			int i = 0;
			while(i!=10) i++;
		}
		ADC_Value_Raw = ADC_Value_Raw/5.0;
		tempval3[j] = ADC_Value_Raw;
		ADC_Value_Raw = 0;
//		HAL_Delay(1);
		int i = 0;
		while(i!=1000) i++;
	}
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET); //with load
	/**
	 * Peak Value Calculation
	 */
	int max_i[2],min_i[2];
	for(int j = 0;j <NbrPTs; j++)
	{
		ADC_Value[3] = tempval3[j]/4095.0*3.38;
		if(ADC_Value[3]>temp_max[3])
		{
			temp_max[3]=ADC_Value[3];
			if(min_i[0]< j) max_i[0] =j;
			else if(min_i[0]<j) max_i[1]=j;
		}
		else if(ADC_Value[3]<temp_min[3]) 
		{
			temp_min[3]=ADC_Value[3];
		}
	}
	/**
	 * DC Bias Calculation
	 */
	temp_bias[3] = (temp_max[3]+temp_min[3])/2.0;
	for(int j = 1; j < 4; j++){temp_max[j] = 0;temp_min[j] = 3.38;}	
	/* Uop Process*/
	Uo_bias = temp_bias[3];
	/**
	 * Read Uop from adc with load resistance
	 */
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET); //without load
	HAL_Delay(500);
	for(int j=0;j<NbrPTs;j++)
	{
		for(int i = 0; i<5; i++)
		{
			ADC_Value_Raw += uhADCxConvertedValue[4];
			int i = 0;
			while(i!=10) i++;
		}
		ADC_Value_Raw = ADC_Value_Raw/5.0;
		tempval3[j] = ADC_Value_Raw;
		ADC_Value_Raw = 0;
//		HAL_Delay(1);
		int i = 0;
		while(i!=1000) i++;
	}
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET); //without load
	/**
	 * Peak Value Calculation
	 */
	for(int j = 0;j <NbrPTs; j++)
	{
		ADC_Value[3] = tempval3[j]/4095.0*3.38;
		if(ADC_Value[3]>temp_max[3])temp_max[3]=ADC_Value[3];
		else if(ADC_Value[3]<temp_min[3]) temp_min[3]=ADC_Value[3];
	}
	/**
	 * DC Bias Calculation
	 */
	temp_bias[3] = (temp_max[3]+temp_min[3])/2.0;
	for(int j = 1; j < 4; j++){temp_max[j] = 0;temp_min[j] = 3.3;}	
	/* Uop Process*/
	Uop_bias = temp_bias[3];


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
		pdata[i] = 0.6/3.25*4096*arm_sin_f32(PI2*i/64.0) + 2048;
	}
}
/**
  * @brief Calculate Values of a Sinewave
	& @param * @pdata pointer of the data @datasize size of the pointer
  * @retval None
  */
float ADC_Value_Raw;
//int i;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	if (htim->Instance == htim4.Instance)
	{
		
//		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
//		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6))
//			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
//		else
//			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
//		for(int i = 0; i<20; i++)
//			ADC_Value_Raw += uhADCxConvertedValue[0];
//		ADC_Value[1] = ADC_Value_Raw/20.0;
//		ADC_Value_Raw = 0;
//		for(int i = 0; i<20; i++)
//			ADC_Value_Raw += uhADCxConvertedValue[2];
//		ADC_Value[2] = ADC_Value_Raw/20.0;
//		ADC_Value_Raw = 0;
//		for(int i = 0; i<20; i++)
//			ADC_Value_Raw += uhADCxConvertedValue[4];
//		ADC_Value[3] = ADC_Value_Raw/20.0;
//		ADC_Value_Raw = 0;
//		
//		ADC_Value[1] = ADC_Value[1]/4095.0*3.3;
//		ADC_Value[2] = ADC_Value[2]/4095.0*3.3;
//		ADC_Value[3] = ADC_Value[3]/4095.0*3.3;
//		ADC_Value[1] = (uhADCxConvertedValue[0]/4095.0)*3.3;
//		ADC_Value[2] = (uhADCxConvertedValue[2]/4095.0)*3.3;
//		ADC_Value[3] = (uhADCxConvertedValue[4]/4095.0)*3.3;
//		Signal_Process();
//		Result_CalValue();
//		tempval[i] = uhADCxConvertedValue[0];
//		i++;
//		if(i == 128) i=0;
	}
}
/**
  * @brief GetValue From adc channel
  * @retval None
  */
uint16_t ADC_GetValue(ADC_HandleTypeDef *hadc)
{
  uint16_t adc_val;
	HAL_ADC_PollForConversion(hadc, 0);
//	if ((HAL_ADC_GetState(hadc) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG)
	{
		HAL_ADC_Start_DMA(&hadc1,ADCConvertedValue,3);
		adc_val = HAL_ADC_GetValue(hadc);
	}

  return adc_val;
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
