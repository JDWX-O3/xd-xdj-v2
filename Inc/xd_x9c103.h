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
#ifndef __XD_X9C103_H
#define __XD_X9C103_H

#ifdef __cplusplus
extern "C" {
#endif



//x9c103Ƭѡ���͵�ƽ��Ч
#define X9C103_UP_DOWN_SET(mco_Value)        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,mco_Value)                //�̵����ߵ�ƽ

#define X9C103_INC_SET(mco_Value)        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,mco_Value)                //�̵����ߵ�ƽ


//x9c103Ƭѡ���͵�ƽ��Ч
#define X9C103_CS_SET(mco_Value)        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,mco_Value)                //�̵����ߵ�ƽ


int32_t X9C103_Value_Add( uint32_t incVal);
int32_t X9C103_Value_Clear(void);
int32_t X9C103_Value_Dec( uint32_t incVal);






#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
