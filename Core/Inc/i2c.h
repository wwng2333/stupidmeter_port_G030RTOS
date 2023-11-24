/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    i2c.h
 * @brief   This file contains all the function prototypes for
 *          the i2c.c file
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>	

  /* USER CODE BEGIN Includes */

  /* USER CODE END Includes */

  /* USER CODE BEGIN Private defines */

#define BIG_LITTLE_SWAP16(x) ((((*(uint16_t *)&x) & 0xff00) >> 8) | \
                              (((*(uint16_t *)&x) & 0x00ff) << 8))

void INA226_Init(void);
void TMP1075_Init(void);
float TMP1075_ReadTemp(void);

void INA226_Update(void);
void INA226_Read_Voltage(void);
void INA226_Read_Current(void);

uint16_t INA226_Read_2Byte(uint8_t addr);
uint16_t TMP1075_Read_2Byte(uint8_t addr);
void INA226_Write_2Byte(uint8_t addr, uint16_t dat);

uint8_t I2C_ReadByte(uint8_t addr);
void I2C_WriteByte(uint8_t addr, uint8_t dat);

void I2C_Start();
void I2C_SendData(uint8_t dat);
void I2C_RecvACK();
void I2C_SendACK();
uint8_t I2C_RecvData();
void I2C_SendNAK();
void I2C_Stop();

  /* USER CODE END Private defines */

  /* USER CODE BEGIN Prototypes */

  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */
