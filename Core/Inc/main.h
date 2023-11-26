/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g0xx_ll_adc.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_rtc.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_RTT.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#pragma pack(1)
	typedef enum cmd_type_enum
	{
		CMD_NONE = 0,
		GET_DATA_CMD = 0xBB,
		UPDATE_RTC_CMD = 0xBC,
		CMD_TYPE_MAX = 255
	} cmd_type_enum;

	typedef struct ina226_info_struct
	{
		float Voltage;
		float Current;
		float Power;
		uint8_t Direction; //if Direction=1, current is negative.
	} ina226_info_struct;

	typedef enum uart_rcv_state_enum
	{
		RCV_IDLE = 0,
		RCV_HEAD,
		RCV_LEN,
		RCV_CMD,
		RCV_DATA,
		RCV_STATE_MAX = 255
	} uart_rcv_state_enum;

	typedef struct uart_cmd_struct
	{
		uint8_t head;
		uint8_t data_len;
		uint8_t cmd;
		uint8_t tail;
	} uart_cmd_struct;
	
	typedef struct rtc_update_struct
	{
		uint8_t head;
		uint8_t data_len;
		uint8_t cmd;
		uint32_t time;
		uint8_t tail;
	} rtc_update_struct;
	
	typedef struct sensor_data_struct
	{
		float max;
		float avg;
		float min;
	} sensor_data_struct;
	
	typedef struct uart_data_struct
	{
		uint8_t head;
		uint8_t data_len;
		sensor_data_struct voltage;
		sensor_data_struct current;
		sensor_data_struct power;
		sensor_data_struct temperature;
		float mWh;
		float mAh;
		float MCU_VCC;
		uint8_t tail;
	} uart_data_struct;
#pragma pack()
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define __Crazy_DEBUG 1
#define UART_BUF_LEN	48
#define UART_RCV_DONE_FLAG (1U << 0)
#define UART_TX_START_FLAG (1U << 1)
#define SIZE(temp) sizeof(temp) / sizeof(uint8_t)
#define HEAD 0x55
#define TAIL 0xAA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
