/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "main.h"
#include "cmsis_os2.h"
#include <stdbool.h>

#define I2C_WRITE 0
#define I2C_READ 1
//#define INA226_ADDR 0x40
#define INA226_ADDR (0x40 << 1)
#define TMP1075_ADDR (0x48 << 1)
#define READ_STRETCH_TIMEOUT	100
#define WRITE_STRETCH_TIMEOUT	80

void I2C_Delay(void)
{
  __IO uint16_t i;
  for (i = 0; i < 8; i++)
    ;
	//osDelay(1);
}

#define I2C_GPIO_PORT GPIOA
#define Pin_SCL LL_GPIO_PIN_12
#define Pin_SDA LL_GPIO_PIN_11

#define Pin_SCL_L LL_GPIO_ResetOutputPin(I2C_GPIO_PORT, Pin_SCL)
#define Pin_SCL_H LL_GPIO_SetOutputPin(I2C_GPIO_PORT, Pin_SCL)

#define Pin_SDA_L LL_GPIO_ResetOutputPin(I2C_GPIO_PORT, Pin_SDA)
#define Pin_SDA_H LL_GPIO_SetOutputPin(I2C_GPIO_PORT, Pin_SDA)

#define Read_SDA_Pin LL_GPIO_IsInputPinSet(I2C_GPIO_PORT, Pin_SDA)
#define Read_SCL_Pin LL_GPIO_IsInputPinSet(I2C_GPIO_PORT, Pin_SCL)

/**
  * @brief Read TMP1075 ID, Set Configuration 
	* Register and Calibration Register
  * @param  none
  * @retval none
  */
void TMP1075_Init(void)
{
	uint16_t id = 0;
	do {
		id = TMP1075_Read_2Byte(0x0F);
		#if __Crazy_DEBUG
		SEGGER_RTT_SetTerminal(1);
		SEGGER_RTT_printf(0, "read TMP1075 0x0F=0x%x\r\n", id);
		#endif
		osDelay(100);
	} while(id != 0x7500);
}

/**
  * @brief Read INA226 ID, Set Configuration 
	* Register and Calibration Register
  * @param  none
  * @retval none
  */
void INA226_Init(void)
{
	uint16_t id = 0;
	do {
		id = INA226_Read_2Byte(0xFE);
		#if __Crazy_DEBUG
		SEGGER_RTT_SetTerminal(1);
		SEGGER_RTT_printf(0, "read INA226 0xFE=0x%x\r\n", id);
		#endif
		osDelay(100);
	} while(id != 0x5449);
	I2C_Write_2Byte(0x00, 0x45FF); // Configuration Register
	I2C_Write_2Byte(0x05, 0x1400); // Calibration Register, 5120, 0.1mA
	//LSB=0.0002mA,R=0.005R Cal=0.00512/(0.0002*0.005)=5120=0x1400
	#ifdef __Crazy_DEBUG
	SEGGER_RTT_printf(0, "set INA226 0x05=0x%x\r\n", 0x1400);
	SEGGER_RTT_SetTerminal(0);
	#endif
}

uint16_t INA226_Read_2Byte(uint8_t addr)
{
	uint16_t dat = 0;
	I2C_Start();
	I2C_SendData(INA226_ADDR);
	I2C_RecvACK();
	I2C_SendData(addr);
	I2C_RecvACK();
	I2C_Start();
	I2C_SendData(INA226_ADDR + 1);
	I2C_RecvACK();
	dat = I2C_RecvData();
	dat <<= 8;
	I2C_SendACK();
	dat |= I2C_RecvData();
	I2C_SendNAK();
	I2C_Stop();

	return dat;
}

uint16_t TMP1075_Read_2Byte(uint8_t addr)
{
	uint16_t dat = 0;
	I2C_Start();
	I2C_SendData(TMP1075_ADDR);
	I2C_RecvACK();
	I2C_SendData(addr);
	I2C_RecvACK();
	I2C_Start();
	I2C_SendData(TMP1075_ADDR + 1);
	I2C_RecvACK();
	dat = I2C_RecvData();
	dat <<= 8;
	I2C_SendACK();
	dat |= I2C_RecvData();
	I2C_SendNAK();
	I2C_Stop();

	return dat;
}

uint8_t I2C_ReadByte(uint8_t addr)
{
	uint8_t dat = 0;
	I2C_Start();
	I2C_SendData(INA226_ADDR);
	I2C_RecvACK();
	I2C_SendData(addr);
	I2C_RecvACK();

	I2C_Delay();

	I2C_Start();
	I2C_SendData(INA226_ADDR + 1);
	I2C_RecvACK();
	dat = I2C_RecvData();
	I2C_SendNAK();
	I2C_Stop();
	return dat;
}

void I2C_Write_2Byte(uint8_t addr, uint16_t dat)
{
	I2C_Start();
	I2C_SendData(INA226_ADDR);
	I2C_RecvACK();
	I2C_SendData(addr);
	I2C_RecvACK();
	I2C_SendData(dat >> 8);
	I2C_RecvACK();
	I2C_SendData(dat);
	I2C_RecvACK();
	I2C_Stop();
}

void I2C_WriteByte(uint8_t addr, uint8_t dat)
{
	I2C_Start();
	I2C_SendData(INA226_ADDR);
	I2C_RecvACK();
	I2C_SendData(addr);
	I2C_RecvACK();
	I2C_SendData(dat);
	I2C_RecvACK();
	I2C_Stop();
}

void I2C_Start()
{
	Pin_SDA_H;
	Pin_SCL_H;
	I2C_Delay();
	Pin_SDA_L;
	I2C_Delay();
	Pin_SCL_L;
}

void I2C_SendData(uint8_t dat)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		Pin_SCL_L;
		I2C_Delay();
		if (dat & 0x80)
			Pin_SDA_H;
		else
			Pin_SDA_L;
		dat <<= 1;
		Pin_SCL_H;
		I2C_Delay();
	}
	Pin_SCL_L;
}

void I2C_SendACK()
{
	Pin_SCL_L;
	I2C_Delay();
	Pin_SDA_L;
	Pin_SCL_H;
	I2C_Delay();
	Pin_SCL_L;
	Pin_SDA_H;
	I2C_Delay();
}

void I2C_RecvACK()
{
	Pin_SCL_L;
	I2C_Delay();
	Pin_SDA_H;
	Pin_SCL_H;
	I2C_Delay();
	Pin_SCL_L;
	I2C_Delay();
}

uint8_t I2C_RecvData()
{
	uint8_t dat, i;
	for (i = 0; i < 8; i++)
	{
		dat <<= 1;
		Pin_SCL_L;
		I2C_Delay();
		Pin_SCL_H;
		I2C_Delay();
		if (Read_SDA_Pin == 1)
		{
			dat |= 0x01;
		}
	}
	return dat;
}

void I2C_SendNAK()
{
	Pin_SCL_L;
	I2C_Delay();
	Pin_SDA_H;
	Pin_SCL_H;
	I2C_Delay();
	Pin_SCL_L;
	I2C_Delay();
}

void I2C_Stop()
{
	Pin_SDA_L;
	Pin_SCL_H;
	I2C_Delay();
	Pin_SDA_H;
	I2C_Delay();
}