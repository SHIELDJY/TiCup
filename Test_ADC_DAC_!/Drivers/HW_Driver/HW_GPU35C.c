/**
  ******************************************************************************
  * @file           : HW_GPU35C.c
  * @brief          : Hardware support for GPU35C LCD Display Module
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @author         : Li Jiayi
  *
  **********
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private function prototypes -----------------------------------------------*/
void GpuSend(const char * str);

/* Private user code ---------------------------------------------------------*/
/**
  * @brief Send commands to lcd display module
	* @param @str pointer of the commands 
  * @retval None
  */
void GpuSend(const char* str)
{
	uint8_t i=0;
	UART_HandleTypeDef huart2;
	huart2.Instance = USART2;
	while (1)
	{	if (str[i]!=0)
		{	HAL_UART_Transmit(&huart2, &str[i], 1, 0xFFFF); 
			i++;
		}
		else return;
	}
}

/**
  * @brief Initialize lcd display module
	* @param None
  * @retval None
  */
void LCD_Initialize(void)
{
	printf("BOXF(0,0,480,320,15);BPIC(1,180,90,1);BPIC(2,130,0,2);SBC(15);DS24(0,0,'System Initializing......',0);\r\n");
}

/**
  * @brief Clear LCD display module with background picture
	* @param None
  * @retval None
  */
void LCD_Clear(void)
{
	GpuSend("CLS(0);SBC(15);BOXF(0,0,480,320,15);BPIC(1,180,90,1);BPIC(2,130,0,2);\r\n");
}

/**
  * @brief Display characters with LCD
	* @param @str string @x range 0 to 480 @y range 0 to 320
  * @retval None
  */
void LCD_PrintChar(const char* str, uint16_t x, uint16_t y)
{
	// GpuSend("CLS(0);BOXF(0,0,480,320,15);BPIC(1,180,90,1);BPIC(2,130,0,2);");
	snprintf(str,"DS24(%d,%d,'%s',0);",x,y,str);
	GpuSend(str);
	GpuSend("\r\n");
}
/**
  * @brief Display uint16_t with LCD
	* @param @uint16_t num @x range 0 to 480 @y range 0 to 320
  * @retval None
  */
void LCD_PrintUint16_t(uint16_t num, uint16_t x, uint16_t y)
{
	char* str;
	// GpuSend("CLS(0);BOXF(0,0,480,320,15);BPIC(1,180,90,1);BPIC(2,130,0,2);");
	snprintf(str,"DS24(%d,%d,'%d',0);",x,y,num);
	GpuSend(str);
	GpuSend("\r\n");
}
