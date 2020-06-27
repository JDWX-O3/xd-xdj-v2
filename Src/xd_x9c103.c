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
#include "main.h"
#include "xd_x9c103.h"
#include "xd_dig_led.h"


void X9C103_delay(void)
{
	uint8_t t = 20;
	while(t--);
	return;
}



//x9c103片选，低电平有效
int32_t X9C103_Value_Clear(void )
{ 
	int i = 0; 
	X9C103_CS_SET(GPIO_PIN_RESET); 
	

	// clear
	X9C103_UP_DOWN_SET(GPIO_PIN_RESET); 
	for(i = 0; i < 100; i++) { 
		X9C103_delay(); 
		X9C103_INC_SET(GPIO_PIN_SET);   
		X9C103_delay(); 
		X9C103_INC_SET(GPIO_PIN_RESET); 
	}

	return 0;
}





//x9c103片选，低电平有效
int32_t X9C103_Value_Add( uint32_t incVal)
{ 
	int i = 0; 
	X9C103_CS_SET(GPIO_PIN_RESET); 
	

	// set 1
	X9C103_UP_DOWN_SET(GPIO_PIN_SET); 
	for(i = 0; i < incVal ; i++) { 
		X9C103_delay(); 
		X9C103_INC_SET(GPIO_PIN_SET);     
		X9C103_delay(); 
		X9C103_INC_SET(GPIO_PIN_RESET);     
	}
	
	return 0;
}


//x9c103片选，低电平有效
int32_t X9C103_Value_Dec( uint32_t incVal)
{ 
	int i = 0; 
	X9C103_CS_SET(GPIO_PIN_RESET); 
	

	// set down 
	X9C103_UP_DOWN_SET(GPIO_PIN_RESET); 
	for(i = 0; i < incVal ; i++) { 
		HAL_Delay(1); 
		X9C103_INC_SET(GPIO_PIN_SET);     
		HAL_Delay(1); 
		X9C103_INC_SET(GPIO_PIN_RESET);     
	}

	return 0;
}



