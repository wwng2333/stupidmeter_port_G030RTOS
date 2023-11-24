/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32g0xx_it.h"
#include "cmsis_os2.h"
#include "usart.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern uint8_t uart_rcv_count;
extern uint8_t uart_rcv_buf[48];
extern uart_rcv_state_enum uart_state;
cmd_type_enum uart_cmd_type;
extern uint8_t uart_rcv_len;
extern osEventFlagsId_t uart_event_flagID;
extern uint8_t uart_rcv_flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
//void SVC_Handler(void)
//{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
//}

/**
  * @brief This function handles Pendable request for system service.
  */
//void PendSV_Handler(void)
//{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
//}

/**
  * @brief This function handles System tick timer.
  */
//void SysTick_Handler(void)
//{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
//}

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
			if (PWR_ON_ACK_CMD == temp || PWR_OFF_ACK_CMD == temp)
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

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
