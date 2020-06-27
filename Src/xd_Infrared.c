/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : xd_infrared.c
  * @brief          : xd_infrared program body
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
#include "xd_infrared.h"

/*

NEC协议的数据格式：引导码、地址码（用户码）、地址反码（用户反码）、键码(数据码)、键码反码
1):引导码由9ms高电平+4.5ms低电平组成。
2):重复码由9ms低电平+2.5ms高电平+0.56ms低电平+97.94ms高电平组成
3):逻辑1（560us的低电平+1680us高电平），逻辑0（560us低电平+560us高电平）。

*/


/**
  * @brief System Clock Configuration
  * @retval None
  */
extern TIM_HandleTypeDef htim5;
T4_InfraredDev  T5Infra_S, *T5Infra = &T5Infra_S;
 
 
void IR_InfraInit(void)
{
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
	__HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);  //使能更新中断

	T5Infra->TriPolarity = 0;
	T5Infra->FrameStart = 0;
}
 
 
//改变TIM5 CHANNEL4的捕获极性
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity)
{
	uint16_t tmpccer = 0;

	tmpccer = TIMx->CCER;
	tmpccer &= (uint16_t)~((uint16_t)TIM_CCER_CC4P);
	tmpccer |= (uint16_t)(TIM_OCPolarity << 12);

	TIMx->CCER = tmpccer;
}
 
 
//捕获中断处理函数
void IR_InfraPeriodHandle(TIM_HandleTypeDef *htim)
{
	T5Infra->TriPolarity = 0;
	T5Infra->FrameStart = 0;
	T5Infra->TriTime[0] = 0;
	T5Infra->TriTime[1] = 0;
	TIM_OC4PolarityConfig(TIM5, TIM_INPUTCHANNELPOLARITY_BOTHEDGE);	
	T5Infra->keepIndex = 0;
	T5Infra->press_count = 0;
	if (T5Infra->keepLevelTime[1] > 0){
		//memset(T5Infra->keepLevelTime, 0, sizeof(T5Infra->keepLevelTime));
		//memset(T5Infra->keepLevel, 0, sizeof(T5Infra->keepLevel));
	}
}
 
 
//捕获中断处理函数
void IR_InfraCaptureHandle(TIM_HandleTypeDef *htim)
{
	//T5Infra->TriPolarity = !T5Infra->TriPolarity;

	/************************** 切换极性以及捕获电平持续时间 **************************/
	 if(T5Infra->TriPolarity == TIM_INPUTCHANNELPOLARITY_RISING)  //上升沿触发
	{
		//TIM_OC4PolarityConfig(TIM5, TIM_INPUTCHANNELPOLARITY_FALLING);	
		T5Infra->TriPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
		T5Infra->keepLevelTime[T5Infra->keepIndex] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_2);	
		T5Infra->keepLevel[T5Infra->keepIndex] = 0;
	}
	else  //下降沿触发
	{
		//TIM_OC4PolarityConfig(TIM5, TIM_INPUTCHANNELPOLARITY_RISING);
		T5Infra->TriPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		T5Infra->keepLevelTime[T5Infra->keepIndex] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_2);
		T5Infra->keepLevel[T5Infra->keepIndex] = 1;
	}

	T5Infra->keepIndex++;
	if (T5Infra->keepIndex > 120){
		T5Infra->keepIndex = 0;
	}

	TIM5->CNT = 0; 	//清除TIM5计数值

	/**************************** 判断同步码以及重复码 ***************************/
	/*
	if(RangJudge(T5Infra->TriTime[0],8500,9500) &&  RangJudge(T5Infra->TriTime[1],4000,5000) ) 
	{
		//同步码
		T5Infra->KEY_VALUE = 0;  
		T5Infra->KEY_Count = 0;  
		T5Infra->DataBit = 0; 
		T5Infra->FrameStart = 1;	//开始接收帧
	}		
	else if(RangJudge(T5Infra->TriTime[0],8500,9500) &&  RangJudge(T5Infra->TriTime[1],2000,3000) ) 
	{
	   //连发码
		 if(++T5Infra->KEY_Count > 250) T5Infra->KEY_Count = 250;
	}
	*/
	/********************************* 接收数据 *********************************/		
	/*
	if( T5Infra->FrameStart == 1) 
	{
		if(RangJudge(T5Infra->TriTime[0],450,650) &&  RangJudge(T5Infra->TriTime[1],450,650) ) 
		{
		   T5Infra->KEY_VALUE &= ~(1<T5Infra->DataBit++);
		}
		else if(RangJudge(T5Infra->TriTime[0],450,650) && RangJudge(T5Infra->TriTime[1],1450,1750) ) 
		{
		   T5Infra->KEY_VALUE |= 1<<T5Infra->DataBit++;
		}

	}	
	*/
	if (T5Infra->keepIndex > 33 && T5Infra->keepLevelTime[0] > 500 &&  RangJudge(T5Infra->keepLevelTime[1], 2000, 3000)){
	
		T5Infra->KEY_VALUE = IR_LevelTime_2_UintNum(&T5Infra->keepLevelTime[2], 32);

		if(T5Infra->KEY_VALUE == 0xb847ff00 && T5Infra->press_count < 1){
			BTN1_PRESS_DOWN_Handler(NULL);
			T5Infra->press_count++;
		}
		else if(T5Infra->KEY_VALUE == 0xbb44ff00 && T5Infra->press_count < 1){
			BTN2_PRESS_DOWN_Handler(NULL);
			T5Infra->press_count++;
		}
		else if(T5Infra->KEY_VALUE == 0xf807ff00){
			BTN3_PRESS_DOWN_Handler(NULL);
			T5Infra->press_count++;
		}
		else if(T5Infra->KEY_VALUE == 0xe916ff00 && T5Infra->press_count < 1){
			BTN4_PRESS_DOWN_Handler(NULL);
			T5Infra->press_count++;
		}
		else if(T5Infra->KEY_VALUE == 0xf30cff00 && T5Infra->press_count < 1){
			BTN5_PRESS_DOWN_Handler(NULL);
			T5Infra->press_count++;
		}
		
	}
	
		
}
 

//获取按键值
uint32_t  IR_LevelTime_2_UintNum(uint16_t  *p_levelTime,  int len)
{
	int i = 0;
	uint32_t ret = 0;
	
	if (len > 32){
		return 0;
	}
	
	//通过与反码异或，验证按键码的正确性
	for(i = 0; i < 32; i++){
	
		if(RangJudge(p_levelTime[i],450,650) ) 
		{
		   ret &= ~(1<< i);
		}
		else if(RangJudge(p_levelTime[i],1070,1170) ) 
		{
		   ret|= 1<<i;
		}
		else{
				return 0;
		}
	}

	return ret;
}

 
//获取按键值
uint8_t  GetInfraredKey(void)
{
	//通过与反码异或，验证按键码的正确性
	if( (uint8_t)( (T5Infra->KEY_VALUE >> 24) ^ (T5Infra->KEY_VALUE >> 16) ) == 0xff  ){
		return (uint8_t)(T5Infra->KEY_VALUE>>16);
	}
	else{
		return 0;
	}
}
 
 
//获取地址
uint8_t  GetInfraredAddr(void)
{
	if( (uint8_t)( (T5Infra->KEY_VALUE ) ^ (T5Infra->KEY_VALUE >> 8) ) == 0xff  ){
		return (uint8_t)(T5Infra->KEY_VALUE);
	}
	else{
		return 0;
	}
}
 



 
//捕获中断
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
 
    if(htim->Instance == TIM5)
	{
		IR_InfraCaptureHandle(htim);
	}
}

