/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.c
 * @brief   This file provides code for the configuration
 *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "string.h"
#include "cmsis_os2.h"

/* USER CODE BEGIN 0 */
extern osMutexId_t uart_mutexId;
extern uint8_t uart_rcv_count;
extern uint8_t uart_rcv_buf[48];
extern uart_rcv_state_enum uart_state;
cmd_type_enum uart_cmd_type;
extern uint8_t uart_rcv_len;
extern osEventFlagsId_t uart_event_flagID;
extern uint8_t uart_rcv_flag;
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */
void UART2_Transmit8_mutex(uint8_t *p, uint8_t counter)
{
  for (uint8_t i = 0; i < counter; i++)
  {
    LL_USART_TransmitData8(USART2, *(p++));
    while (!LL_USART_IsActiveFlag_TXE(USART2));
  }
}

void UART2_Transmit8(uint8_t *p, uint8_t counter)
{
  osMutexAcquire(uart_mutexId, osWaitForever);
  UART2_Transmit8_mutex(p, counter);
  osMutexRelease(uart_mutexId);
}

void UART2_SendString(const char* str)
{
    /* 遍历字符串中的每个字符 */
    for (int i = 0; str[i] != '\0'; i++)
    {
        /* 确保USART已经准备好发送数据 */
        while (!LL_USART_IsActiveFlag_TXE(USART2))
        {
        }
        /* 发送一个字符 */
        LL_USART_TransmitData8(USART2, (uint8_t)str[i]);
    }

    /* 确保所有数据都已经发送出去 */
    while (!LL_USART_IsActiveFlag_TC(USART2))
    {
    }
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  if (LL_USART_IsActiveFlag_RXNE(USART2))
  {
    uint8_t temp = LL_USART_ReceiveData8(USART2);
    if (uart_state == RCV_HEAD && temp == HEAD && uart_rcv_count == 0)
    {
      uart_rcv_buf[uart_rcv_count] = temp;
      uart_state = RCV_LEN;
      uart_rcv_count++;
    }
    else if (uart_state == RCV_LEN)
    {
			/* 确认接收长度是否小于接收缓冲区,如果否重置接收流程 */
			if (temp <= UART_BUF_LEN)
			{
				uart_rcv_buf[uart_rcv_count] = temp;
				uart_rcv_len = temp - 2;
				uart_state = RCV_CMD;
				uart_rcv_count++;
			}
			else
			{
				//memset(uart_rcv_buf, 0, SIZE(uart_rcv_buf));
				uart_state = RCV_HEAD;
				uart_rcv_count = 0;
				uart_rcv_len = 0;
			}
    }
    else if (uart_state == RCV_CMD)
    {
			if (GET_DATA_CMD == temp || UPDATE_RTC_CMD == temp)
			{
				uart_rcv_buf[uart_rcv_count] = temp;
				uart_rcv_len--;
				uart_cmd_type = temp;
				uart_state = RCV_DATA;
				uart_rcv_count++;
			}
			else
			{
				/* 非法指令,复位并重新接收 */
				uart_state = RCV_HEAD;
				uart_rcv_count = 0;
				uart_rcv_len = 0;
			}
    }
    else if (uart_state == RCV_DATA)
    {
			/* 没到最后一个数据,继续接收 */
      if (uart_rcv_len != 1)
      {
        uart_rcv_len--;
        uart_rcv_buf[uart_rcv_count++] = temp;
      }
			/* 准备接收最后一个数据 */
      else
      { 
				/* 确认是否为最后一个数据 */
        if (uart_rcv_len == 1)
        {
					/* 确认最后一个数据是否为帧尾 */
          if (temp == TAIL)
          {
            uart_rcv_buf[uart_rcv_count++] = temp;
            // 触发UART工作线程干活
            osEventFlagsSet(uart_event_flagID, UART_RCV_DONE_FLAG);
            // osEventFlagsSet(uart_event_flagID, UART_TX_START_FLAG);
          }
					else
					{
						uart_state = RCV_HEAD;
						uart_rcv_count = 0;
						uart_rcv_len = 0;
					}
        }
      }
    }
  }
  LL_USART_ClearFlag_ORE(USART2);
}

/* USER CODE END 1 */
