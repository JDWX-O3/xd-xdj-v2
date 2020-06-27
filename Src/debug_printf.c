
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_iwdg.h"
#include "xd_dig_led.h"
#include "xd_tm1650.h"
#include "xd_key.h"
#include "xd_relay.h"
#include "xd_infrared.h"
#include "xd_x9c103.h"
#include "xd_sensor.h"
#include "multi_button.h"
#include "debug_printf.h"
#include <stdio.h>
#include <string.h>


extern UART_HandleTypeDef huart4;


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{

    //具体哪个串口可以更改huart1为其它串口
    HAL_UART_Transmit(&huart4 , (uint8_t *)&ch, 1 , 0xffff);
    return ch;
}



