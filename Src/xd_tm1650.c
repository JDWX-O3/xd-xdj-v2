/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
//#include "stm32f1xx.h"
#include "xd_tm1650.h"

const uint8_t DIG_LED_NUM[16]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,   0x77,0x7c,0x39,0x5e,0x79,0x71};//数码管显示0~9对应的值
//带小数点
const uint8_t DIG_LED_P_NUM[16]={0xbf,0x86,0xdb,0xcf,0xe6,0xed,0xfd,0x87,0xff,0xef,   0x77,0x7c,0x39,0x5e,0x79,0x71};//数码管显示0~9对应的值


#if 1
//#include "tb_delay.h"
//#include "i2c.h"
void I2C_delay(void)
{
	uint8_t t = 20;
	while(t--);
	return;
}
int I2C_Start(void)
{
	I2C_SDA_OUT();
	
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)
	{
		return DISABLE;
	}
	SDA_L;
	I2C_delay();
	if(SDA_read)
	{
		return DISABLE;
	}
	SCL_L;
	return ENABLE;
}
void I2C_Stop(void)
{
	I2C_SDA_OUT();	
	SCL_L;
	SDA_L;	
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
}
 
static void I2C_Ack()
{
	SCL_L;
	I2C_SDA_OUT();	
	
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
}
 
static void I2C_NoAck()
{
	SCL_L;
	I2C_SDA_OUT();
	
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
}
uint8_t I2C_GetAck(void)
{
  uint8_t time = 0;
	I2C_SDA_IN();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	while(SDA_read)
	{
		time++;
		if(time > 250)
		{			
			SCL_L;
			return DISABLE;
		}
	}
	SCL_L;
	return ENABLE;
}
void I2C_SendByte(uint8_t Data)
{
	uint8_t cnt;
	I2C_SDA_OUT();	
	for(cnt=0; cnt<8; cnt++)
	{
		SCL_L;                              
		I2C_delay();

		if(Data & 0x80)
		{
			SDA_H;                         
		}
		else
		{
			SDA_L;                         
		}
		Data <<= 1;
		SCL_H;                              
		I2C_delay();
	}
	SCL_L;                                   
	I2C_delay();
}
 
 
uint8_t I2C_ReadByte(uint8_t ack)
{
  uint8_t cnt;
  uint16_t data;
  I2C_SDA_IN();	
	
  for(cnt=0; cnt<8; cnt++)
  {
	    SCL_L;                                
	    I2C_delay();
			
	    SCL_H;                             
	    data <<= 1;
	    if(SDA_read)
	    {
	      data |= 0x01;                              
	    }
	    I2C_delay();
	  }
	  if(ack == 1)
	  {
	     I2C_NoAck();
	  }
	  else
	  {
	     I2C_Ack();
	  }
	  return data;                                  
}
void I2C_GPIO_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//__HAL_RCC_GPIOB_CLK_ENABLE();

	
	GPIO_InitStructure.Pin = I2C_Pin_SCL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.Pin = I2C_Pin_SDA;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}
 
 
void I2C_SDA_IN()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.Pin = I2C_Pin_SDA;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT; 
	GPIO_InitStructure.Pull = GPIO_PULLUP; 
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);	
}
 
void I2C_SDA_OUT()
{
   GPIO_InitTypeDef  GPIO_InitStructure;
   GPIO_InitStructure.Pin = I2C_Pin_SDA;
   GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP; 
   GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;	
   HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);	
}
void I2C_Initializes(void)
{
	I2C_GPIO_Configuration();
	I2C_SDA_OUT();	
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
}

/**********************************************/
int32_t TM1650_Open_Display(void)
{
  	I2C_Start(); 
	I2C_SendByte(0x48);     //模式命令

	I2C_GetAck();
	I2C_SendByte(0x1);

	I2C_Stop();
	return 1;
}

int32_t TM1650_Close_Display(void)
{
  	I2C_Start(); 
	I2C_SendByte(0x48);

	I2C_GetAck();
	I2C_SendByte(0x0);
	I2C_Stop();
	return 1;
}


int32_t TM1650_First_Display_Num_p(uint8_t Data, uint8_t dp_flag)
{
	if(Data > (sizeof(DIG_LED_NUM) -1)){
		return -1;
	}

  	I2C_Start(); 
	I2C_SendByte(0x6a);

	I2C_GetAck();
	if (dp_flag){
		I2C_SendByte(DIG_LED_P_NUM[Data]);
	}
	else{
		I2C_SendByte(DIG_LED_NUM[Data]);
	}	I2C_Stop();
	return 0;
}



int32_t TM1650_Second_Display_Num_p(uint8_t Data, uint8_t dp_flag)
{
	if(Data > (sizeof(DIG_LED_NUM) -1)){
		return -1;
	}
  	I2C_Start(); 
	I2C_SendByte((0x6c));

	I2C_GetAck();
	if (dp_flag){
		I2C_SendByte(DIG_LED_P_NUM[Data]);
	}
	else{
		I2C_SendByte(DIG_LED_NUM[Data]);
	}
	I2C_Stop();
	return 0;
}







int32_t TM1650_Thirth_Display_Num_p(uint8_t Data, uint8_t dp_flag)
{
	if(Data > (sizeof(DIG_LED_NUM) -1)){
		return -1;
	}
  	I2C_Start(); 
	I2C_SendByte(0x6e);

	I2C_GetAck();
	if (dp_flag){
		I2C_SendByte(DIG_LED_P_NUM[Data]);
	}
	else{
		I2C_SendByte(DIG_LED_NUM[Data]);
	}	I2C_Stop();
	return 0;
}

//第四个没有使用
int32_t TM1650_Forth_Display_Num_p(uint8_t Data, uint8_t dp_flag)
{
	if(Data > (sizeof(DIG_LED_NUM) -1)){
		return -1;
	}
  	I2C_Start(); 
	I2C_SendByte(0x68);

	I2C_GetAck();
	if (dp_flag){
		I2C_SendByte(DIG_LED_P_NUM[Data]);
	}
	else{
		I2C_SendByte(DIG_LED_NUM[Data]);
	}
	I2C_Stop();
	return 0;
}




int32_t TM1650_First_Display_Num(uint8_t Data)
{
	return TM1650_First_Display_Num_p(Data, 0);
}

int32_t TM1650_Second_Display_Num(uint8_t Data)
{
	return TM1650_Second_Display_Num_p(Data, 0);
}

int32_t TM1650_Thirth_Display_Num(uint8_t Data)
{
	return TM1650_Thirth_Display_Num_p(Data, 0);
}

int32_t TM1650_Forth_Display_Num(uint8_t Data)
{
	return TM1650_Forth_Display_Num_p(Data, 0);
}



//第四个没有使用
int32_t TM1650_Display_Num(uint32_t Data)
{
	int b0,b1,b2;

	if (Data > 999){
		return -1;
	}

	b2=Data/100;
	b1=(Data-b2*100)/10;
	b0=Data%10;

	TM1650_First_Display_Num(b2);
	TM1650_Second_Display_Num(b1);
	TM1650_Thirth_Display_Num(b0);
	return 0;
}


int32_t TM1650_Display_Float_2_Num(float Data)
{
	int b0,b1;

	if (Data > 99){
		return -1;
	}


	if (Data < 10){
		b0=(int)Data;
		b1=((int)(Data* 10))%10;
		TM1650_Second_Display_Num_p(b0, 1);
	}
	else{
		b0=((int)Data)/10;
		b1=((int)Data)%10;
		TM1650_Second_Display_Num_p(b0, 0);
	}

	TM1650_Thirth_Display_Num(b1);
	return 0;
}

#endif





