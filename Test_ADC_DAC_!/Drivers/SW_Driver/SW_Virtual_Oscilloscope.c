/**
  ******************************************************************************
  * @file           : SW_Virtual_OSC.c
  * @brief          : Software support file of virtual oscilloscope
  ******************************************************************************
  * @author					: Li Jiayi
  *
  ******************************************************************************
  */
	
#include "main.h"

uint8_t begincode[2] = {0x03,0xfc};
uint8_t endcode[2] = {0xfc,0x03};

void VOSC_SendOne_uint16(UART_HandleTypeDef *huart, uint16_t data)
{
	uint8_t a,a2;	
	HAL_UART_Transmit(huart, begincode, 2, 100);
	a=(data>>8)&0xff;a2=data&0xff;	
	HAL_UART_Transmit(huart, &a2, 1, 100);
	HAL_UART_Transmit(huart, &a, 1, 100);
  HAL_UART_Transmit(huart, endcode, 2, 100);
}

void VOSC_SendTwo_uint16(UART_HandleTypeDef *huart, uint16_t data1, uint16_t data2)
{
	uint8_t a,a2;	
	HAL_UART_Transmit(huart, begincode, 2, 100);
	a=(data1>>8)&0xff;a2=data1&0xff;	
	HAL_UART_Transmit(huart, &a2, 1, 100);
	HAL_UART_Transmit(huart, &a, 1, 100);
	a=(data2>>8)&0xff;a2=data2&0xff;	
	HAL_UART_Transmit(huart, &a2, 1, 100);
	HAL_UART_Transmit(huart, &a, 1, 100);
  HAL_UART_Transmit(huart, endcode, 2, 100);
}
