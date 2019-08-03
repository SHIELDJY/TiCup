/**
  ******************************************************************************
  * @file           : HW_GPU35C.h
  * @brief          : Headers for GPU35C HW_GPU35C.h
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @author         : Li Jiayi
  *
  **********
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_GPU35C_H
#define __HW_GPU35C_H

#include "stm32f4xx_hal.h"
#include <stddef.h>

/* Exported functions prototypes ---------------------------------------------*/
void LCD_Initialize(void);
void LCD_Clear(void);
void LCD_PrintChar(const char* str, uint16_t x, uint16_t y);
void LCD_PrintUint16_t(uint16_t num, uint16_t x, uint16_t y);
#endif /* __HW_GPU35C_H */
