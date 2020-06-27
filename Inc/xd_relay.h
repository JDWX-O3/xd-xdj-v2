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
#ifndef __XD_RELAY_H
#define __XD_RELAY_H

#ifdef __cplusplus
extern "C" {
#endif


/* 定义控制 SDA SCL 的宏 HAL库版    */
#define RELAY_OUT_1_SET        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET)         //继电器高电平
#define RELAY_OUT_1_RESET        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET)            //继电器低电平

#define RELAY_OUT_2_SET        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET)                //继电器高电平
#define RELAY_OUT_2_RESET        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET)            //继电器低电平

#define RELAY_OUT_3_SET        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET)                //继电器高电平
#define RELAY_OUT_3_RESET        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET)            //继电器低电平

#define RELAY_FAN_RUN   FAN_Realy_Start
#define RELAY_FAN_STOP FAN_Realy_Stop


#define RELAY_O3_RUN    O3_Realy_Start
#define RELAY_O3_STOP   O3_Realy_Stop



int32_t FAN_Realy_Start(void);
int32_t FAN_Realy_Stop(void);
int32_t O3_Realy_Start(void);
int32_t O3_Realy_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
