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
#ifndef __XD_DIG_LED_H
#define __XD_DIG_LED_H

#ifdef __cplusplus
extern "C" {
#endif

void LED_A_BLUE_Off(void);
void LED_A_BLUE_On(void);

 
void LED_B_GREEN_On(void);
void LED_B_GREEN_Off(void);



 
void LED_C_GREEN_On(void);
void LED_C_GREEN_Off(void);


 
void LED_D_GREEN_On(void);
void LED_D_GREEN_Off(void);

 


void LED_E_GREEN_Off(void);
void LED_E_GREEN_On(void);






void LED_F_YELLOW_On(void);
void LED_F_YELLOW_Off(void);





void LED_G_YELLOW_On(void);
void LED_G_YELLOW_Off(void);

 


void LED_H_YELLOW_On(void);
void LED_H_YELLOW_Off(void);





void LED_I_RED_On(void);
void LED_I_RED_Off(void);

 




void LED_J_RED_On(void);
void LED_J_RED_Off(void);

 


void LED_A_BLUE_Flash(void);
void LED_J_Flash(void);



void X9C103_G1_LED_open(void);
void X9C103_G1_LED_close(void);
void X9C103_G2_LED_open(void);
void X9C103_G2_LED_close(void);
void X9C103_G3_LED_open(void);
void X9C103_G3_LED_close(void);
void X9C103_Gear_Display(uint8_t x9c103_gear);



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
