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
#include "xd_tm1650.h"
#include "xd_key.h"
#include "xd_relay.h"
#include "xd_infrared.h"
#include "xd_x9c103.h"
#include "xd_sensor.h"
#include "main.h"
#include "multi_button.h"
#include "xd_sensor.h"


extern ADC_HandleTypeDef hadc1;


/**
  * @brief System Clock Configuration
  * @retval None
  */
// ����һ
 

uint32_t ADC_ReadData(void)
{
	static uint32_t adc_value;
	//static uint32_t cur_value;

	// ����ADCת��
	HAL_ADC_Start(&hadc1);
	// �ȴ�ת����ɣ��ڶ���������ʾ��ʱʱ�䣬��λms
	HAL_ADC_PollForConversion(&hadc1, 100);
	// Ϊ��ȡADC״̬


	adc_value = HAL_ADC_GetValue(&hadc1);
	// ��ȡ���Ҷ������2��12�η����ο���ѹΪ3.3V
	//voltage = (float)adc_value / 4096 * 3.3;
	//HAL_Delay(1000);
	return adc_value;
}



