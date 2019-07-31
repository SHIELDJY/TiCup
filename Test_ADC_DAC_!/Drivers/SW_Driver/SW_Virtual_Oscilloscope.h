/**
  ******************************************************************************
  * @file           : SW_Virtual_Oscilloscope.h
  * @brief          : Header for SW_Virtual_Oscilloscope.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @author         : Li Jiayi
  *
  **********
*/
#include "stm32f4xx_hal.h"
#include <stddef.h>

void VOSC_SendOne_uint16(UART_HandleTypeDef *huart, uint16_t data);
void VOSC_SendTwo_uint16(UART_HandleTypeDef *huart, uint16_t data1, uint16_t data2);