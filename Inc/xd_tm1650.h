/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __XD_TM1650_H
#define __XD_TM1650_H

#ifdef __cplusplus
extern "C" {
#endif

/**********************************

TM1650芯片控制20路灯函数

PB7为SCL口

PB6为SDA口

**********************************/
#if 1
#define I2C_Pin_SCL     GPIO_PIN_6
#define I2C_Pin_SDA     GPIO_PIN_7
/*********************************************/
 
 

/* 定义控制 SDA SCL 的宏 HAL库版    */
#define SDA_H        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SDA,GPIO_PIN_SET)                //SDA高电平
#define SDA_L        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SDA,GPIO_PIN_RESET)            //SDA低电平
#define SCL_H        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SCL,GPIO_PIN_SET)              //SCL高电平
#define SCL_L        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SCL,GPIO_PIN_RESET)            //SCL低电平



/* 获取 SDA SCL 引脚状态，标准库版也是用相同逻辑    */
#define SCL_read        HAL_GPIO_ReadPin(GPIOB ,I2C_Pin_SCL)        //获取SDA引脚状态
#define SDA_read        HAL_GPIO_ReadPin(GPIOB ,I2C_Pin_SDA)        //获取SDA引脚状态



int I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
void I2C_SDA_OUT(void);
void I2C_SDA_IN(void);
uint8_t I2C_GetAck(void);
void I2C_SendByte(uint8_t Data);
uint8_t I2C_ReadByte(uint8_t ack);
void I2C_delay(void);
void I2C_Initializes(void);




int32_t TM1650_Open_Display(void);
int32_t TM1650_Close_Display(void);
int32_t TM1650_First_Display_Num(uint8_t Data);
int32_t TM1650_Second_Display_Num(uint8_t Data);
int32_t TM1650_Thirth_Display_Num(uint8_t Data);
//int32_t TM1650_Forth_Display_Num(uint8_t Data);
int32_t TM1650_Display_Num(uint32_t Data);
int32_t TM1650_Display_Float_2_Num(float Data);


#endif


#if 0
/* 定义控制 SDA SCL 的宏 标准库版
#define I2C_SDA_UP        GPIO_SetBits  (GPIOB,GPIO_PIN_8)    //SDA高电平
#define I2C_SDA_LOW        GPIO_ResetBits(GPIOB,GPIO_PIN_8)    //SDA低电平
#define I2C_SCL_UP        GPIO_SetBits  (GPIOB,GPIO_PIN_9)    //SCL高电平
#define I2C_SCL_LOW        GPIO_ResetBits(GPIOB,GPIO_PIN_9)    //SCL低电平
*/

#define I2C_Pin_SCL     GPIO_PIN_6
#define I2C_Pin_SDA     GPIO_PIN_7

/* 定义控制 SDA SCL 的宏 HAL库版    */

#define I2C_SDA_UP        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SDA,GPIO_PIN_SET)                //SDA高电平
#define I2C_SDA_LOW        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SDA,GPIO_PIN_RESET)            //SDA低电平
#define I2C_SCL_UP        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SCL,GPIO_PIN_SET)              //SCL高电平
#define I2C_SCL_LOW        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SCL,GPIO_PIN_RESET)            //SCL低电平



/* 获取 SDA SCL 引脚状态，标准库版也是用相同逻辑    */
#define I2C_SDA        HAL_GPIO_ReadPin(GPIOB ,I2C_Pin_SDA)        //获取SDA引脚状态
#define I2C_SCL        HAL_GPIO_ReadPin(GPIOB ,I2C_Pin_SCL)        //获取SCL引脚状态


void         i2c_start(void);                    //*    开始信号
void         i2c_ack(void);                        //*    应答信号
void         I2C_No_ack(void);                    //*    非应答信号
void         i2c_stop(void);                        //*    停止信号
char         i2c_wit_ack(void);                //*    等待应答
void         i2c_send(uint8_t dat);        //* 发送数据
uint8_t i2c_read(void);                        //* 接收数据

void SDA_OUT(void);                //*    SDA 设置为输出
void SDA_IN(void);                //*    SDA 设置为输入

void delay_us(uint32_t i);        //*延时




int32_t TM1650_Open_Display(void);
int32_t TM1650_Close_Display(void);
int32_t TM1650_First_Display_Num(uint8_t Data);
int32_t TM1650_Second_Display_Num(uint8_t Data);
int32_t TM1650_Thirth_Display_Num(uint8_t Data);
//int32_t TM1650_Forth_Display_Num(uint8_t Data);

 #endif

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
