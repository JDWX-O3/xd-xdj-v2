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
#ifndef __XD_SENSOR_H
#define __XD_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif


/* 定义控制 SDA SCL 的宏 HAL库版    */
#define READ_KQ_SENSOR_A()        (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0))        //继电器高电平
#define READ_KQ_SENSOR_B()        (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1))          //继电器低电平

uint32_t ADC_ReadData(void);



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
