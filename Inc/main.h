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
	FSM_FAN_RUN          = 1,    /*!<��������0s-30s��״̬     */
	FSM_FAN_RUN_OVER_30S,    /*!<��������30s���״̬     */
	FSM_O3_RUN,    /*!< o3 ����0-30s��״̬    */
	FSM_O3_RUN_OVER_30S,    /*!< O3 ����30s���״̬    */
	FSM_O3_STOP,    /*!< O3ֹͣ0-30s��״̬     */
	FSM_O3_STOP_OVER_30S,    /* O3ͣ30s ���״̬     */
	FSM_FAN_STOP,    /*! ����ֹͣ0s-30s��״̬    */
	FSM_FAN_STOP_OVER_30S,    /*!<����ֹͣ30s���״̬     */
	FSM_FAULT_STATUS,    /*!<����ֹͣ30s���״̬     */
}DEV_MANAUL_RUN_STATUS_TypeDef;

#define SAMPLE_NUM   8


/// Attributes structure for thread.
typedef struct {

	uint32_t                 device_fsm_status;   ///�ֶ�����״̬
	uint32_t                 current_cmd;   ///��ǰ�豸ָ� run��stop	
	uint32_t                 press_key_value;   ///<��Դ��bits
	//uint32_t                 debug_mode;   ///debug mode
	uint32_t                 fault_code;   ///����״̬

	uint32_t                 power_key_status;   ///<��Դ��bits
	uint32_t                 dev_run_mode;   ///�豸��ģʽ
	uint32_t                 tm1650_status;   ///�Զ����ֶ��л�


	//uint32_t                 fan_run_cmd;   ///��������״̬
	uint32_t                 fan_run_status;   ///��������״̬
	uint32_t                 fan_run_sec;   ///��������30��
	uint32_t                 fan_stop_sec;   ///��������30��


	//uint32_t                 o3_run_cmd;   ///��������״̬
	uint32_t                 o3_run_status;   ///o3 ����״̬
	uint32_t                 o3_set_minute;   ///�û��յ����õĶ�ʱ
	uint32_t                 o3_run_sec;   ///o3 ����30��
	uint32_t                 o3_stop_sec;   ///��������30��

	//����ֹͣ���Ӽ������ò���������auto ģʽ
	uint32_t                 fan_stop_minute;   ///����ֹͣ
	uint32_t                 o3_run_minute;   /// ��������



	uint32_t                 x9c103_gear;   ///��λ ��0-9��


	uint32_t                 sec_count;   ///�û���ʱ������λ��
	uint32_t                 user_timer_max;   ///�û���ʱ�����ֵ����λ��
	float                      current_ck_max;   ///����������ֵ����λ��


	uint8_t                 kq_sensor_a;   ///
	uint8_t                 kq_sensor_b;   ///
	uint8_t                 kq_quality;   ///
	uint8_t                 resv;   ///
	
	uint32_t               kq_adjust;   ///  run ����
	uint32_t               kq_adjust_count;   ///  stop����

	uint32_t               cur_adc_value;   ///  ����ֵ
	float                    cur_sw_value;   ///  ����ֵ
	float                    a_cur_value[SAMPLE_NUM];   ///  ����ֵ
	
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

//1:��Ҫ�������  0:����Ҫ�������
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
