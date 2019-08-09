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
char buf[128];
/* Private user code ---------------------------------------------------------*/
/**
  * @brief Send commands to lcd display module
	* @param @str pointer of the commands 
  * @retval None
  */
void GpuSend(const char* str)
{
	uint8_t i=0;
	UART_HandleTypeDef huart1;
	huart1.Instance = USART1;
	printf(str);
//	while (1)5
//	{	if (str[i]!=0)
//		{	HAL_UART_Transmit(&huart1, &str[i], 1, 0xFFFF); 
//			i++;
//		}
//		else return;
//	}
}

/**
  * @brief Initialize lcd display module
	* @param None
  * @retval None
  */
void LCD_Initialize(void)
{
	printf("BOXF(0,0,480,320,15);BPIC(1,180,90,1);BPIC(2,130,0,2);SBC(15);PS24(0,0,0,'\xB3\xF5\xCA\xBC\xBB\xAF\xD6\xD0',0);\r\n");
}

/**
  * @brief Clear LCD display module with background picture
	* @param None
  * @retval None
  */
void LCD_Clear(void)
{
	GpuSend("CLS(15);\r\n");
	HAL_Delay(50);
	GpuSend("BPIC(1,180,90,1);\r\n");
	HAL_Delay(50);
	GpuSend("BPIC(2,130,0,2);\r\n");
}

/**
  * @brief Display characters with LCD
	* @param @str string @x range 0 to 480 @y range 0 to 320
  * @retval None
  */
void LCD_PrintChar(const char* str, uint16_t x, uint16_t y)
{
	sprintf(buf,"PS24(0,%d,%d,'%s',0);\r\n",x,y,str);
	GpuSend(buf);
}
/**
  * @brief Display uint16_t with LCD
	* @param @uint16_t num @x range 0 to 480 @y range 0 to 320
  * @retval None
  */
void LCD_PrintUint16_t(uint16_t num, uint16_t x, uint16_t y)
{
	sprintf(buf,"PS24(0,%d,%d,'%d',0);\r\n",x,y,num);
	GpuSend(buf);
}

/**
  * @brief Display float with LCD
	* @param @num float number @x range 0 to 480 @y range 0 to 320
  * @retval None
  */
void LCD_PrintFloat(float num, uint16_t x, uint16_t y)
{
	sprintf(buf,"PS24(0,%d,%d,'%.3f',0);\r\n",x,y,num);
	GpuSend(buf);
}
/**
  * @brief LCD Plot initialize 
	* @param @num float number @x range 0 to 480 @y range 0 to 320
  * @retval None
	* @warning @Unfinished!!!!!!!!!!!
	* @warning @Unfinished!!!!!!!!!!!
	* @warning @Unfinished!!!!!!!!!!!
  */
void LCD_PlotInit(void)
{
	GpuSend("DQX(2,180,5,20,63,5);\r\n");
}
/**
  * @brief Plot point with LCD
	* @param @num float number
  * @retval None
	* @warning @Unfinished!!!!!!!!!!!
	* @warning @Unfinished!!!!!!!!!!!
	* @warning @Unfinished!!!!!!!!!!!
  */
void LCD_PlotPoint(uint8_t data)
{
	sprintf(buf,"S%d;\r\n",data);
	GpuSend(buf);
}

/**
  * @brief Plot figure with LCD
	* @param @data uint8_t pointer contains 64 points
  * @retval None
	* @warning @Unfinished!!!!!!!!!!!
	* @warning @Unfinished!!!!!!!!!!!
	* @warning @Unfinished!!!!!!!!!!!
  */
void LCD_Plot(uint8_t* data)
{
	uint8_t i;
	char str[7];
	for(i=0;i<64;i++)
	{
		sprintf(str,"S%d;\r\n",data[i]);
		GpuSend(str);
	}
}

