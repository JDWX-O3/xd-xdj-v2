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
#ifndef __XD_INFRARED_H
#define __XD_INFRARED_H

#ifdef __cplusplus
extern "C" {
#endif

#define  RangJudge(val,min,max)   (val>min? (val<max? 1 : 0) : 0)

 

 

typedef struct T4_Infrared
{
	uint32_t  KEY_VALUE;   //记录32位地址以及按键码
	uint16_t  TriTime[2];  //记录电平持续时间 0：低电平  1:高电平
	uint8_t   DataBit;
	uint8_t   FrameStart;
	uint8_t   KEY_Count;
	uint16_t   TriPolarity;

	uint16_t  keepLevelTime[128];  //记录电平持续时间 0：低电平  1:高电平
	uint16_t  keepLevel[128];  //记录电平持续时间 0：低电平  1:高电平
	uint16_t  keepIndex;  //记录电平持续时间 0：低电平  1:高电平
	
	uint16_t   press_count;

	uint8_t   temp_test;
}T4_InfraredDev;

 

 

extern T4_InfraredDev  *T5Infra;

 

 

 

void IR_InfraInit(void);
void IR_InfraPeriodHandle(TIM_HandleTypeDef *htim);
void IR_InfraCaptureHandle(TIM_HandleTypeDef *htim);

 

uint8_t  GetInfraredKey(void);
uint8_t  GetInfraredAddr(void);
uint32_t  IR_LevelTime_2_UintNum(uint16_t  *p_levelTime,  int len);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
