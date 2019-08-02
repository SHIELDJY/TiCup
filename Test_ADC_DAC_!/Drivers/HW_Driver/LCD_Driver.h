/**
  ******************************************************************************
  * @file           : LCD_Driver.c
  * @brief          : Hardware Driver for LCD Display
	* @pin						:	GND   	电源地
	*										VCC   	接5V或3.3v电源
	*										SCL   	接PA5（SCL）
	*										SDA   	接PA7（SDA）
	*										RES   	接PB0
	*										DC    	接PB1
	*										CS    	接PA4 
	*										BL			接PB10
	*
  ******************************************************************************
  */
/**
  ******************************************************************************
  * @file           : LCD_Driver.h
  * @brief          : Header for LCD_Driver.h file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define X_MAX_PIXEL	        128
#define Y_MAX_PIXEL	        160

#define RED  	0xf800
#define GREEN	0x07e0
#define BLUE 	0x001f
#define WHITE	0xffff
#define BLACK	0x0000
#define YELLOW  0xFFE0
#define GRAY0   0xEF7D   	    //灰色0 3165    0011    0001    0110    0101
#define GRAY1   0x8410      	//灰色1 0000    0000    0000    0000
#define GRAY2   0x4208      	//灰色2 1111    1111    1101    1111

#define LCD_CTRLA   	  	GPIOA		//定义TFT数据端口
#define LCD_CTRLB   	  	GPIOB		//定义TFT数据端口

#define LCD_SCL        	GPIO_Pin_5	//PB13--->>TFT --SCL/SCK
#define LCD_SDA        	GPIO_Pin_7	//PB15 MOSI--->>TFT --SDA/DIN
#define LCD_CS        	GPIO_Pin_4  //MCU_PB11--->>TFT --CS/CE

#define LCD_LED        	GPIO_Pin_10  //MCU_PB9--->>TFT --BL
#define LCD_RS         	GPIO_Pin_1	//PB11--->>TFT --RS/DC
#define LCD_RST     	GPIO_Pin_0	//PB10--->>TFT --RST

//#define LCD_CS_SET(x) LCD_CTRL->ODR=(LCD_CTRL->ODR&~LCD_CS)|(x ? LCD_CS:0)

//液晶控制口置1操作语句宏定义
#define	LCD_SCL_SET  	LCD_CTRLA->BSRR=LCD_SCL    
#define	LCD_SDA_SET  	LCD_CTRLA->BSRR=LCD_SDA   
#define	LCD_CS_SET  	LCD_CTRLA->BSRR=LCD_CS  

    
#define	LCD_LED_SET  	LCD_CTRLB->BSRR=LCD_LED   
#define	LCD_RS_SET  	LCD_CTRLB->BSRR=LCD_RS 
#define	LCD_RST_SET  	LCD_CTRLB->BSRR=LCD_RST 
//液晶控制口置0操作语句宏定义
#define	LCD_SCL_CLR  	LCD_CTRLA->BRR=LCD_SCL  
#define	LCD_SDA_CLR  	LCD_CTRLA->BRR=LCD_SDA 
#define	LCD_CS_CLR  	LCD_CTRLA->BRR=LCD_CS 
    
#define	LCD_LED_CLR  	LCD_CTRLB->BRR=LCD_LED 
#define	LCD_RST_CLR  	LCD_CTRLB->BRR=LCD_RST
#define	LCD_RS_CLR  	LCD_CTRLB->BRR=LCD_RS 

#define LCD_DATAOUT(x) LCD_DATA->ODR=x; //数据输出
#define LCD_DATAIN     LCD_DATA->IDR;   //数据输入

#define LCD_WR_DATA(data){\
LCD_RS_SET;\
LCD_CS_CLR;\
LCD_DATAOUT(data);\
LCD_WR_CLR;\
LCD_WR_SET;\
LCD_CS_SET;\
} 


/* Exported functions prototypes ---------------------------------------------*/
void LCD_GPIO_Init(void);
void Lcd_WriteIndex(uint8_t Index);
void Lcd_WriteData(uint8_t Data);
void Lcd_WriteReg(uint8_t Index,uint8_t Data);
uint16_t Lcd_ReadReg(uint8_t LCD_Reg);
void Lcd_Reset(void);
void Lcd_Init(void);
void Lcd_Clear(uint16_t Color);
void Lcd_SetXY(uint16_t x,uint16_t y);
void Gui_DrawPoint(uint16_t x,uint16_t y,uint16_t Data);
uint16_t Lcd_ReadPoint(uint16_t x,uint16_t y);
void Lcd_SetRegion(uint16_t x_start,u16 y_start,uint16_t x_end,uint16_t y_end);
void LCD_WriteData_16Bit(uint16_t Data);

#endif /* __LCD_DRIVER_H */
