/**
  ******************************************************************************
  * @file           : SW_fft.c
  * @brief          : Software support file of fast fourier transform
  ******************************************************************************
  * @author					: Li Jiayi
  *
  ******************************************************************************
  */

#include "arm_math.h"
#include "arm_const_structs.h"

#define NPT  1024  //1024点FFT
#define Fs 5120  //采样频率 5120Hz 频率分辨率 5Hz
#define PI2 6.28318530717959
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
}
