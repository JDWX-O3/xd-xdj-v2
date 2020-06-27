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
#ifndef __XD_KEY_H
#define __XD_KEY_H

#ifdef __cplusplus
extern "C" {
#endif





#define READ_GPIO_KEY_1()   (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5))
#define READ_GPIO_KEY_2()   (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4))
#define READ_GPIO_KEY_3()   (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))
#define READ_GPIO_KEY_4()   (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))
#define READ_GPIO_KEY_5()   (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))


void XD_Key_init(void);
void BTN1_PRESS_DOWN_Handler(void* btn);
void BTN2_PRESS_DOWN_Handler(void* btn);
void BTN3_PRESS_DOWN_Handler(void* btn);
void BTN4_PRESS_DOWN_Handler(void* btn);
void BTN5_PRESS_DOWN_Handler(void* btn);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
