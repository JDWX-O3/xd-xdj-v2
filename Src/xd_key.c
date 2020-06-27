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




extern userOpAttr_t g_config;
struct Button btn1;
struct Button btn2;
struct Button btn3;
struct Button btn4;
struct Button btn5;



uint8_t READ_GPIO_button_1(void) 
{
    return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
}

uint8_t READ_GPIO_button_2(void) 
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
}
uint8_t READ_GPIO_button_3(void) 
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
}
uint8_t READ_GPIO_button_4(void) 
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
}
uint8_t READ_GPIO_button_5(void) 
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
// ����һ
 
void BTN1_PRESS_DOWN_Handler(void* btn)
{
	g_config.press_key_value = 1;
	
	//��ͣ����
	g_config.power_key_status = !g_config.power_key_status;
	if (g_config.power_key_status == ENABLE){
		// ��Դ������
		LED_J_RED_On();
		g_config.dev_run_mode = RUN_MOD_MANAUL;  //Ĭ���ֶ�ģʽ


		g_config.tm1650_status = ENABLE;
		TM1650_Open_Display();
		TM1650_Open_Display();



		GConfig_Init();
		DEVICE_RUN;
		g_config.o3_set_minute = 0;
		TM1650_Display_Num(0);


		//������λĬ��2
		if (g_config.x9c103_gear != 1){
			X9C103_Value_Clear();
			g_config.x9c103_gear = 1;
			X9C103_Value_Add(10);
			X9C103_Gear_Display(g_config.x9c103_gear);
		}
		//��ʱ30��
		//osTimerStart(myTimer01Handle, 30000);
	}
	else{
		g_config.tm1650_status = DISABLE;
		TM1650_Close_Display();
		TM1650_Close_Display();
		
		//��������
		g_config.fault_code = 0;


		DEVICE_STOP;
		//��������Ϊ0
		g_config.o3_set_minute = 0;
 		//������λĬ��1
		//���ʳ�
		X9C103_Value_Clear();
		g_config.x9c103_gear = 0;
		X9C103_Gear_Display(g_config.x9c103_gear);	
		
		//osTimerStart(myTimer02Handle, 60000);
	}
}

void BTN1_PRESS_UP_Handler(void* btn)
{
    //do something...
}


void BTN2_PRESS_DOWN_Handler(void* btn)
{
	//g_config.press_key_value = 2;
	
	//������
	if(g_config.power_key_status == DISABLE){
		return;
	}

	//ģʽ�л�
	if(g_config.dev_run_mode == RUN_MOD_MANAUL){
		//g_config.o3_set_minute = 888;
		g_config.dev_run_mode = RUN_MOD_AUTO;			
		TM1650_First_Display_Num(0xa);
		TM1650_Second_Display_Num(0);
		TM1650_Thirth_Display_Num(g_config.kq_quality);


		//������λĬ��2
		if (g_config.x9c103_gear != 2){
			X9C103_Value_Clear();
			g_config.x9c103_gear = 2;
			X9C103_Value_Add(35);
			X9C103_Gear_Display(g_config.x9c103_gear);
		}
		
	}
	else if(g_config.dev_run_mode == RUN_MOD_AUTO){
		g_config.dev_run_mode = RUN_MOD_MANAUL;
		g_config.o3_set_minute = 0;
		TM1650_Display_Num(g_config.o3_set_minute);

		//������λĬ��1
		if (g_config.x9c103_gear != 1){
			X9C103_Value_Clear();
			g_config.x9c103_gear = 1;
			X9C103_Value_Add(25);
			X9C103_Gear_Display(g_config.x9c103_gear);
		}
		
	}
}


void BTN3_PRESS_DOWN_Handler(void* btn)
{
	g_config.press_key_value = 3;
	
	//��ʱ����ÿ��һ����ô1 ��
	if(g_config.power_key_status == DISABLE){
		return;
	}

	//ģʽ�л�
	if(g_config.dev_run_mode == RUN_MOD_MANAUL){
		g_config.sec_count = 0;  //��ֹ����
		g_config.o3_set_minute++;
		if (g_config.o3_set_minute > g_config.user_timer_max){
			g_config.o3_set_minute = 0;
		}


		// �ֶ�ģʽ�£�����ֹͣ�����Ӽ�������������
		if (g_config.fan_run_status != DEV_RUN && g_config.o3_set_minute > 0){
			DEVICE_RUN;               //�豸��ʼ����
			g_config.fan_run_sec = 1;  //��ֹͣ��
			g_config.o3_stop_sec = 60;
		}
		
		TM1650_Display_Num(g_config.o3_set_minute);
	}
	else{

		return;
	}



}


void BTN4_PRESS_DOWN_Handler(void* btn)
{
	g_config.press_key_value = 4;
	
	//��λ�����ֵ�λ��
	//TM1650_Thirth_Display_Num(g_config.x9c103_gear);

	//������
	if(g_config.power_key_status == DISABLE){
		return;
	}


	//ģʽ�л�
	if(g_config.dev_run_mode == RUN_MOD_MANAUL){
		g_config.x9c103_gear++;
		if ( g_config.x9c103_gear > 3){
			//ѭ��
			X9C103_Value_Clear();
			g_config.x9c103_gear = 1;
		}	

		////
		if (g_config.x9c103_gear == 1){
			X9C103_Value_Add(25);
			X9C103_Gear_Display(g_config.x9c103_gear);
		}
		else if (g_config.x9c103_gear == 2){
			X9C103_Value_Add(35);  //35
			X9C103_Gear_Display(g_config.x9c103_gear);
		}
		else if (g_config.x9c103_gear == 3){
			X9C103_Value_Add(40);    //50
			X9C103_Gear_Display(g_config.x9c103_gear);
		}

	}
	else{

		return;
	}


}



void BTN5_PRESS_DOWN_Handler(void* btn)
{
	g_config.press_key_value = 5;
	
	if(g_config.power_key_status == DISABLE){
		return;
	}


	//
	if(g_config.dev_run_mode == RUN_MOD_MANAUL){
	
		//ȡ��
		//DEVICE_STOP;
		g_config.o3_set_minute = 0;

		TM1650_Display_Num(g_config.o3_set_minute);

		//������λĬ��1
		X9C103_Gear_Display(0);	
		//���ʳ�
		X9C103_Value_Clear();

		g_config.x9c103_gear = 0;
	}
	else{

		return;
	}


}

void BTN3_LONG_PRESS_HOLD_Handler(void* btn)
{
	HAL_Delay(40);
	BTN3_PRESS_DOWN_Handler(btn);
}

void LONG_PRESS_Debug_Handler(void* btn)
{

	if(g_config.power_key_status == ENABLE || g_config.press_key_value != 5){
		return;
	}

	//debug mode
	g_config.dev_run_mode = RUN_MOD_DEBUG;
	g_config.tm1650_status = ENABLE;
	DEVICE_STOP;
	g_config.o3_set_minute = 0;




	/*
	if(g_config.power_key_status == DISABLE || g_config.dev_run_mode == RUN_MOD_MANAUL){
		return;
	}

	if (g_config.fault_code == 0){
		g_config.fault_code = 1;
	}
	else{
		g_config.fault_code = 0;
	}
	*/
}



void XD_Key_init(void)
{
	button_init(&btn1, READ_GPIO_button_1, 0);  //��ʼ�����󶨰���GPIO��ƽ��ȡ�ӿ�
	button_init(&btn2, READ_GPIO_button_2, 0);
	button_init(&btn3, READ_GPIO_button_3, 0);
	button_init(&btn4, READ_GPIO_button_4, 0);
	button_init(&btn5, READ_GPIO_button_5, 0);
	
	button_attach(&btn1, LONG_PRESS_START,       BTN1_PRESS_DOWN_Handler);
	button_attach(&btn1, PRESS_UP,         BTN1_PRESS_UP_Handler);
	
	button_attach(&btn2, PRESS_DOWN,       BTN2_PRESS_DOWN_Handler);
	button_attach(&btn3, PRESS_DOWN,       BTN3_PRESS_DOWN_Handler);
	button_attach(&btn4, PRESS_DOWN,       BTN4_PRESS_DOWN_Handler);
	button_attach(&btn5, PRESS_DOWN,       BTN5_PRESS_DOWN_Handler);
	//button_attach(&btn1, PRESS_REPEAT,     BTN1_PRESS_REPEAT_Handler);
	//button_attach(&btn1, SINGLE_CLICK,     BTN1_SINGLE_Click_Handler);
	//button_attach(&btn1, DOUBLE_CLICK,     BTN1_DOUBLE_Click_Handler);
	button_attach(&btn3, LONG_PRESS_HOLD, BTN3_LONG_PRESS_HOLD_Handler);
	
	button_attach(&btn2, LONG_PRESS_START,  LONG_PRESS_Debug_Handler);
	button_start(&btn1);
	button_start(&btn2);
	button_start(&btn3);
	button_start(&btn4);
	button_start(&btn5);
}





