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

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */
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

/* USER CODE END 1 */
