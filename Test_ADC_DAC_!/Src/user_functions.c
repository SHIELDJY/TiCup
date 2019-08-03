/**
  ******************************************************************************
  * @file           : user_functions.c
  * @brief          : Main program body
  ******************************************************************************
  */

/* Private variables ---------------------------------------------------------*/
float32_t fftout[1024];
float32_t fftout_amp[1024];
float32_t  testInput_f32[NPT];
float32_t  testOutput_f32[NPT];
float32_t  testOutput[NPT];

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

    /* ����ʵ�����鲿��ʵ�����鲿..... ��˳��洢���� */ 
  for(i=0; i<1024; i++) 
  { 
    /*3��Ƶ�� 50Hz 2500Hz 2550Hz */ 
    testInput_f32[i] = 1000*arm_sin_f32(PI2*i*50.0/Fs) + 
               2000*arm_sin_f32(PI2*i*2500.0/Fs)  + 
               3000*arm_sin_f32(PI2*i*2550.0/Fs); 
  } 

  /* 1024��ʵ���п���FFT */  
  arm_rfft_fast_f32(&S, testInput_f32, testOutput_f32, ifftFlag); 

  /* Ϊ�˷��������arm_cfft_f32����Ľ�����Աȣ����������1024��ģֵ��ʵ�ʺ���arm_rfft_fast_f32 
     ֻ������512��   
  */  
   arm_cmplx_mag_f32(testOutput_f32, testOutput, fftSize); 
			for(i=0;i<512;i++)
				VOSC_SendOne_uint16(&huart2, (uint16_t)testOutput[i]);

}

/**
  * @brief  DAC ͨ��1 ���ǲ�����
  * @param  None
  * @retval None
  */
static void DAC_Ch1_TriangleConfig(void)
{
  /**-3- ���� DAC ͨ��1 �������ǲ� 
	*/
  if(HAL_DACEx_TriangleWaveGenerate(&hdac, DAC_CHANNEL_1, DAC_TRIANGLEAMPLITUDE_4095) != HAL_OK)
  {
    /* Triangle wave generation Error */
    Error_Handler();
  }
  /**-4- ʹ�� DAC ͨ��1 
	*/  
  if(HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
  /**-5- ���� DAC ͨ��1 DHR12RD �Ĵ���
	*/
  if(HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x100) != HAL_OK)
  {
    /* Setting value Error */
    Error_Handler();
  }
}