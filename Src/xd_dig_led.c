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
#include "xd_dig_led.h"






/**
  * @brief System Clock Configuration
  * @retval None
  */



void LED_A_BLUE_On(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);  //??GPIO??0
}
void LED_A_BLUE_Off(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);    //??GPIO??1
}
 
void LED_A_BLUE_Flash(void)
{
	static char flag = 0;
	if (flag){
		LED_A_BLUE_On();
	}
	else{
		LED_A_BLUE_Off();
	}
	flag=!flag;
}

 
void LED_B_GREEN_On(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);  //??GPIO??0
}
void LED_B_GREEN_Off(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);    //??GPIO??1
}


 
void LED_C_GREEN_On(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);  //??GPIO??0
}
void LED_C_GREEN_Off(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);    //??GPIO??1
}

 
void LED_D_GREEN_On(void)
{ 
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);  //??GPIO??0
}
void LED_D_GREEN_Off(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);    //??GPIO??1
}
 


void LED_E_GREEN_On(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);  //??GPIO??0
}
void LED_E_GREEN_Off(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);    //??GPIO??1
}
 





void LED_F_YELLOW_On(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);  //??GPIO??0
}
void LED_F_YELLOW_Off(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);    //??GPIO??1
}




void LED_G_YELLOW_On(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);  //??GPIO??0
}
void LED_G_YELLOW_Off(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);    //??GPIO??1
}
 


void LED_H_YELLOW_On(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);  //??GPIO??0
}
void LED_H_YELLOW_Off(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);    //??GPIO??1
}




void LED_I_RED_On(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);  //??GPIO??0
}
void LED_I_RED_Off(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);    //??GPIO??1
}
 




void LED_J_RED_On(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  //??GPIO??0
}
void LED_J_RED_Off(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);    //??GPIO??1
}



void LED_J_Flash(void)
{
	static char flag = 0;
	if (flag){
		LED_J_RED_On();
	}
	else{
		LED_J_RED_Off();
	}
	flag=!flag;
}



void X9C103_G3_LED_open(void)
{
		//LED_G_YELLOW_On();
		//LED_H_YELLOW_On();
		LED_I_RED_On();
		LED_J_RED_On();	
}
void X9C103_G3_LED_close(void)
{
		//LED_G_YELLOW_Off();
		//LED_H_YELLOW_Off();
		LED_I_RED_Off();
		LED_J_RED_Off();
}

void X9C103_G2_LED_open(void)
{
		//LED_D_GREEN_On();
		//LED_E_GREEN_On();
		LED_F_YELLOW_On();
		LED_G_YELLOW_On();
		LED_H_YELLOW_On();
}

void X9C103_G2_LED_close(void)
{
		//LED_D_GREEN_Off();
		//LED_E_GREEN_Off();
		LED_F_YELLOW_Off();
		LED_G_YELLOW_Off();
		LED_H_YELLOW_Off();
}


void X9C103_G1_LED_open(void)
{
		LED_A_BLUE_On();
		LED_B_GREEN_On();
		LED_C_GREEN_On();
		LED_D_GREEN_On();
		LED_E_GREEN_On();
}

void X9C103_G1_LED_close(void)
{
		LED_A_BLUE_Off();
		LED_B_GREEN_Off();
		LED_C_GREEN_Off();
		LED_D_GREEN_Off();
		LED_E_GREEN_Off();
}



void X9C103_Gear_Display(uint8_t x9c103_gear)
{
	if (x9c103_gear == 0){
		X9C103_G1_LED_close();
		X9C103_G2_LED_close();
		X9C103_G3_LED_close();
	}
	else if (x9c103_gear == 1){
		X9C103_G1_LED_open();
		X9C103_G2_LED_close();
		X9C103_G3_LED_close();
	}
	else if (x9c103_gear == 2){
		X9C103_G1_LED_open();
		X9C103_G2_LED_open();
		X9C103_G3_LED_close();
	}
	else if (x9c103_gear == 3){
		X9C103_G1_LED_open();
		X9C103_G2_LED_open();
		X9C103_G3_LED_open();
	}
}





