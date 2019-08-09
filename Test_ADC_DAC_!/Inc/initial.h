
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INITIAL_H
#define __INITIAL_H

/* Exported functions prototypes ---------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DAC_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_TIM2_Config(void);
void MX_TIM3_Config(void);
void MX_TIM4_Config(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);

#endif //__INITIAL_H