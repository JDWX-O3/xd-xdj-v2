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

TM1650оƬ����20·�ƺ���

PB7ΪSCL��

PB6ΪSDA��

**********************************/
#if 1
#define I2C_Pin_SCL     GPIO_PIN_6
#define I2C_Pin_SDA     GPIO_PIN_7
/*********************************************/
 
 

/* ������� SDA SCL �ĺ� HAL���    */
#define SDA_H        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SDA,GPIO_PIN_SET)                //SDA�ߵ�ƽ
#define SDA_L        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SDA,GPIO_PIN_RESET)            //SDA�͵�ƽ
#define SCL_H        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SCL,GPIO_PIN_SET)              //SCL�ߵ�ƽ
#define SCL_L        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SCL,GPIO_PIN_RESET)            //SCL�͵�ƽ



/* ��ȡ SDA SCL ����״̬����׼���Ҳ������ͬ�߼�    */
#define SCL_read        HAL_GPIO_ReadPin(GPIOB ,I2C_Pin_SCL)        //��ȡSDA����״̬
#define SDA_read        HAL_GPIO_ReadPin(GPIOB ,I2C_Pin_SDA)        //��ȡSDA����״̬



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
/* ������� SDA SCL �ĺ� ��׼���
#define I2C_SDA_UP        GPIO_SetBits  (GPIOB,GPIO_PIN_8)    //SDA�ߵ�ƽ
#define I2C_SDA_LOW        GPIO_ResetBits(GPIOB,GPIO_PIN_8)    //SDA�͵�ƽ
#define I2C_SCL_UP        GPIO_SetBits  (GPIOB,GPIO_PIN_9)    //SCL�ߵ�ƽ
#define I2C_SCL_LOW        GPIO_ResetBits(GPIOB,GPIO_PIN_9)    //SCL�͵�ƽ
*/

#define I2C_Pin_SCL     GPIO_PIN_6
#define I2C_Pin_SDA     GPIO_PIN_7

/* ������� SDA SCL �ĺ� HAL���    */

#define I2C_SDA_UP        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SDA,GPIO_PIN_SET)                //SDA�ߵ�ƽ
#define I2C_SDA_LOW        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SDA,GPIO_PIN_RESET)            //SDA�͵�ƽ
#define I2C_SCL_UP        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SCL,GPIO_PIN_SET)              //SCL�ߵ�ƽ
#define I2C_SCL_LOW        HAL_GPIO_WritePin(GPIOB,I2C_Pin_SCL,GPIO_PIN_RESET)            //SCL�͵�ƽ



/* ��ȡ SDA SCL ����״̬����׼���Ҳ������ͬ�߼�    */
#define I2C_SDA        HAL_GPIO_ReadPin(GPIOB ,I2C_Pin_SDA)        //��ȡSDA����״̬
#define I2C_SCL        HAL_GPIO_ReadPin(GPIOB ,I2C_Pin_SCL)        //��ȡSCL����״̬


void         i2c_start(void);                    //*    ��ʼ�ź�
void         i2c_ack(void);                        //*    Ӧ���ź�
void         I2C_No_ack(void);                    //*    ��Ӧ���ź�
void         i2c_stop(void);                        //*    ֹͣ�ź�
char         i2c_wit_ack(void);                //*    �ȴ�Ӧ��
void         i2c_send(uint8_t dat);        //* ��������
uint8_t i2c_read(void);                        //* ��������

void SDA_OUT(void);                //*    SDA ����Ϊ���
void SDA_IN(void);                //*    SDA ����Ϊ����

void delay_us(uint32_t i);        //*��ʱ




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
