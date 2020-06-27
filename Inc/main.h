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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */



/** 
  * @brief  HAL DMA Callback ID structure definition
  */
typedef enum
{
	FSM_FAN_RUN          = 1,    /*!<风扇运行0s-30s的状态     */
	FSM_FAN_RUN_OVER_30S,    /*!<风扇运行30s后的状态     */
	FSM_O3_RUN,    /*!< o3 运行0-30s的状态    */
	FSM_O3_RUN_OVER_30S,    /*!< O3 运行30s后的状态    */
	FSM_O3_STOP,    /*!< O3停止0-30s的状态     */
	FSM_O3_STOP_OVER_30S,    /* O3停30s 后的状态     */
	FSM_FAN_STOP,    /*! 风扇停止0s-30s的状态    */
	FSM_FAN_STOP_OVER_30S,    /*!<风扇停止30s后的状态     */
	FSM_FAULT_STATUS,    /*!<风扇停止30s后的状态     */
}DEV_MANAUL_RUN_STATUS_TypeDef;

#define SAMPLE_NUM   8


/// Attributes structure for thread.
typedef struct {

	uint32_t                 device_fsm_status;   ///手动运行状态
	uint32_t                 current_cmd;   ///当前设备指令， run，stop	
	uint32_t                 press_key_value;   ///<电源键bits
	//uint32_t                 debug_mode;   ///debug mode
	uint32_t                 fault_code;   ///故障状态

	uint32_t                 power_key_status;   ///<电源键bits
	uint32_t                 dev_run_mode;   ///设备运模式
	uint32_t                 tm1650_status;   ///自动，手动切换


	//uint32_t                 fan_run_cmd;   ///风扇运行状态
	uint32_t                 fan_run_status;   ///风扇运行状态
	uint32_t                 fan_run_sec;   ///风扇运行30秒
	uint32_t                 fan_stop_sec;   ///风扇运行30秒


	//uint32_t                 o3_run_cmd;   ///风扇运行状态
	uint32_t                 o3_run_status;   ///o3 运行状态
	uint32_t                 o3_set_minute;   ///用户收到设置的定时
	uint32_t                 o3_run_sec;   ///o3 运行30秒
	uint32_t                 o3_stop_sec;   ///风扇运行30秒

	//风扇停止分钟计数，该参数适用于auto 模式
	uint32_t                 fan_stop_minute;   ///风扇停止
	uint32_t                 o3_run_minute;   /// 臭氧运行



	uint32_t                 x9c103_gear;   ///档位 【0-9】


	uint32_t                 sec_count;   ///用户定时器，单位秒
	uint32_t                 user_timer_max;   ///用户定时器最大值，单位秒
	float                      current_ck_max;   ///电流检测最大值，单位秒


	uint8_t                 kq_sensor_a;   ///
	uint8_t                 kq_sensor_b;   ///
	uint8_t                 kq_quality;   ///
	uint8_t                 resv;   ///
	
	uint32_t               kq_adjust;   ///  run 调节
	uint32_t               kq_adjust_count;   ///  stop调节

	uint32_t               cur_adc_value;   ///  电流值
	float                    cur_sw_value;   ///  电流值
	float                    a_cur_value[SAMPLE_NUM];   ///  电流值
	
	uint32_t                  reserved;   ///< reserved (must be 0)
} userOpAttr_t;


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void GConfig_Init(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define digout1_Pin GPIO_PIN_2
#define digout1_GPIO_Port GPIOC
#define IR_Pin GPIO_PIN_1
#define IR_GPIO_Port GPIOA
#define digout2_Pin GPIO_PIN_2
#define digout2_GPIO_Port GPIOA
#define digout3_Pin GPIO_PIN_4
#define digout3_GPIO_Port GPIOA
#define adin2_Pin GPIO_PIN_4
#define adin2_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */
#define KEY1_SCAN_COUNT 4


#define XD_VERSION   2

//1:需要电流检测  0:不需要电流检测
#define CURRENT_P_VALUE   0
//#define CURRENT_P_VALUE   0.35



#define DEV_FAULT   2
#define DEV_RUN   1
#define DEV_STOP   0


#define RUN_MOD_MANAUL   0
#define RUN_MOD_AUTO   1
#define RUN_MOD_FAULT   2
#define RUN_MOD_DEBUG   3


#define DEVICE_RUN  ( g_config.current_cmd = DEV_RUN)
#define DEVICE_STOP    (g_config.current_cmd = DEV_STOP)
#define DEVICE_FAULT   (g_config.current_cmd = DEV_FAULT)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
