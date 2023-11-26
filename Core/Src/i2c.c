/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "main.h"
#include "cmsis_os2.h"
#include <stdbool.h>

extern ina226_info_struct ina226_info;

#define I2C_WRITE 0
#define I2C_READ 1
#define INA226_ADDR (0x40 << 1)
#define TMP1075_ADDR (0x48 << 1)

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

float TMP1075_ReadTemp(void)
{
	float temp = 0.0f;
	int16_t dat = 0;
	dat = TMP1075_Read_2Byte(0x00);
	if(dat&0x8000) 
	{
		dat = ~(dat - 1);
	}
	temp = (float)dat / 256;
	#if __Crazy_DEBUG
	SEGGER_RTT_SetTerminal(1);
	SEGGER_RTT_printf(0, "read TMP1075 0x00=%.2f\r\n", temp);
	#endif
	return temp;
}

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
	} while(id != 0x5449);
	INA226_Write_2Byte(0x00, 0x45FF); // Configuration Register
	INA226_Write_2Byte(0x05, 0x2155); // Calibration Register, 5120, 0.1mA
	//LSB=0.0002mA,R=0.003R Cal=0.00512/(0.0002*0.003)=8533=0x2155
	#ifdef __Crazy_DEBUG
	SEGGER_RTT_printf(0, "set INA226 0x05=0x%x\r\n", 0x2155);
	SEGGER_RTT_SetTerminal(0);
	#endif
}

void INA226_Update(void)
{
	INA226_Read_Voltage();
	osDelay(1);
	INA226_Read_Current();
	osDelay(1);
	ina226_info.Power = ina226_info.Voltage * ina226_info.Current;
}

void INA226_Read_Voltage(void)
{
	uint16_t temp = 0;
	temp = INA226_Read_2Byte(0x02);
	ina226_info.Voltage = 1.25 * (float)INA226_Read_2Byte(0x02) / 1000;
	#ifdef __Crazy_DEBUG
	SEGGER_RTT_SetTerminal(1);
	SEGGER_RTT_printf(0, "read INA226 0x02=%d\r\n", temp);
	SEGGER_RTT_SetTerminal(0);
	#endif
}

void INA226_Read_Current(void)
{
	uint16_t temp = 0;
	temp = INA226_Read_2Byte(0x04);
	if(temp&0x8000) 
	{
		temp = ~(temp - 1);
		ina226_info.Direction = 1;
	}
	else
	{
		ina226_info.Direction = 0;
	}
	ina226_info.Current = (float)temp * 0.2;
	#ifdef __Crazy_DEBUG
	SEGGER_RTT_SetTerminal(1);
	SEGGER_RTT_printf(0, "read INA226 0x04=%d\r\n", temp);
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

void INA226_Write_2Byte(uint8_t addr, uint16_t dat)
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