/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
#include "RTE_Components.h"
#include "i2c.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "Queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
osTimerId_t timer0;
osMutexId_t uart_mutexId;
osThreadId_t app_main_ID;
osThreadId_t uart_transmit_ID;
osEventFlagsId_t uart_event_flagID;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
ina226_info_struct ina226_info = {.Voltage = 0.0f, .Current = 0.0f, .Power = 0.0f, .Direction = 0};
Queue Voltage_queue = {.front = 0, .rear = 0};
Queue Current_queue = {.front = 0, .rear = 0};
Queue Power_queue = {.front = 0, .rear = 0};
Queue Temperature_queue = {.front = 0, .rear = 0};
uart_rcv_state_enum uart_state = RCV_HEAD;
extern cmd_type_enum uart_cmd_type;
uint8_t uart_rcv_buf[UART_BUF_LEN] = {0};
uart_cmd_struct uart_recv_cmd = {0};
uart_data_struct uart_data = {.head = HEAD, .tail = TAIL};
//rtc_update_struct rtc_update = {0};
uint8_t uart_rcv_count = 0;
uint8_t uart_rcv_len = 0;
uint8_t uart_rcv_flag = 0;
float mAh, mWh, MCU_VCC, MCU_Temp = 0.0f;
float time_past = 0.0f;
uint32_t i2c_last_tick;
char str[16];

const uint16_t table[256] = {0x0, 0xa001, 0xe003, 0x4002, 0x6007, 0xc006, 0x8004, 0x2005, 0xc00e, 0x600f, 0x200d, 0x800c, 0xa009, 0x8, 0x400a, 0xe00b, 0x201d, 0x801c, 0xc01e, 0x601f,
                             0x401a, 0xe01b, 0xa019, 0x18, 0xe013, 0x4012, 0x10, 0xa011, 0x8014, 0x2015, 0x6017, 0xc016, 0x403a, 0xe03b, 0xa039, 0x38, 0x203d, 0x803c, 0xc03e, 0x603f,
                             0x8034, 0x2035, 0x6037, 0xc036, 0xe033, 0x4032, 0x30, 0xa031, 0x6027, 0xc026, 0x8024, 0x2025, 0x20, 0xa021, 0xe023, 0x4022, 0xa029, 0x28, 0x402a, 0xe02b,
                             0xc02e, 0x602f, 0x202d, 0x802c, 0x8074, 0x2075, 0x6077, 0xc076, 0xe073, 0x4072, 0x70, 0xa071, 0x407a, 0xe07b, 0xa079, 0x78, 0x207d, 0x807c, 0xc07e, 0x607f,
                             0xa069, 0x68, 0x406a, 0xe06b, 0xc06e, 0x606f, 0x206d, 0x806c, 0x6067, 0xc066, 0x8064, 0x2065, 0x60, 0xa061, 0xe063, 0x4062, 0xc04e, 0x604f, 0x204d, 0x804c,
                             0xa049, 0x48, 0x404a, 0xe04b, 0x40, 0xa041, 0xe043, 0x4042, 0x6047, 0xc046, 0x8044, 0x2045, 0xe053, 0x4052, 0x50, 0xa051, 0x8054, 0x2055, 0x6057, 0xc056,
                             0x205d, 0x805c, 0xc05e, 0x605f, 0x405a, 0xe05b, 0xa059, 0x58, 0xa0e9, 0xe8, 0x40ea, 0xe0eb, 0xc0ee, 0x60ef, 0x20ed, 0x80ec, 0x60e7, 0xc0e6, 0x80e4, 0x20e5,
                             0xe0, 0xa0e1, 0xe0e3, 0x40e2, 0x80f4, 0x20f5, 0x60f7, 0xc0f6, 0xe0f3, 0x40f2, 0xf0, 0xa0f1, 0x40fa, 0xe0fb, 0xa0f9, 0xf8, 0x20fd, 0x80fc, 0xc0fe, 0x60ff,
                             0xe0d3, 0x40d2, 0xd0, 0xa0d1, 0x80d4, 0x20d5, 0x60d7, 0xc0d6, 0x20dd, 0x80dc, 0xc0de, 0x60df, 0x40da, 0xe0db, 0xa0d9, 0xd8, 0xc0ce, 0x60cf, 0x20cd, 0x80cc, 0xa0c9, 0xc8,
                             0x40ca, 0xe0cb, 0xc0, 0xa0c1, 0xe0c3, 0x40c2, 0x60c7, 0xc0c6, 0x80c4, 0x20c5, 0x209d, 0x809c, 0xc09e, 0x609f, 0x409a, 0xe09b, 0xa099, 0x98, 0xe093, 0x4092, 0x90, 0xa091,
                             0x8094, 0x2095, 0x6097, 0xc096, 0x80, 0xa081, 0xe083, 0x4082, 0x6087, 0xc086, 0x8084, 0x2085, 0xc08e, 0x608f, 0x208d, 0x808c, 0xa089, 0x88, 0x408a, 0xe08b, 0x60a7, 0xc0a6,
                             0x80a4, 0x20a5, 0xa0, 0xa0a1, 0xe0a3, 0x40a2, 0xa0a9, 0xa8, 0x40aa, 0xe0ab, 0xc0ae, 0x60af, 0x20ad, 0x80ac, 0x40ba, 0xe0bb, 0xa0b9, 0xb8, 0x20bd, 0x80bc, 0xc0be, 0x60bf,
                             0x80b4, 0x20b5, 0x60b7, 0xc0b6, 0xe0b3, 0x40b2, 0xb0, 0xa0b1};

static const osTimerAttr_t timerAttr_soc_cb = {
    .name = "timer0",
};

static osMutexAttr_t uart_mutex_attr =
    {
        .name = "uart_mutex"};

static const osThreadAttr_t ThreadAttr_app_main =
	{
		.name = "app_main",
		.priority = (osPriority_t)osPriorityNormal,
		.stack_size = 512};

static const osThreadAttr_t ThreadAttr_uart_transmit =
    {
			.name = "uart_transmit",
			.priority = (osPriority_t)osPriorityNormal,
			.stack_size = 512};
		
static const osEventFlagsAttr_t FlagsAttr_uart_event =
    {
        .name = "uart_rcv_evt"};

uint32_t ADC_Read_Voltage(ADC_TypeDef *ADCx)
{
	uint32_t result = 0;
	LL_ADC_Enable(ADCx);
	while (LL_ADC_IsActiveFlag_ADRDY(ADCx) == 0);
	if (LL_ADC_IsActiveFlag_EOC(ADCx)) LL_ADC_ClearFlag_EOC(ADCx);
	//LL_ADC_REG_SetSequencerChannels(ADCx, Channel);
	LL_ADC_REG_StartConversion(ADCx);
	while (LL_ADC_IsActiveFlag_EOC(ADCx) == 0);
	LL_ADC_ClearFlag_EOC(ADCx);
	result = LL_ADC_REG_ReadConversionData12(ADCx);
	SEGGER_RTT_printf(0, "ADC read:%d\r\n", result);
	return result;
}
				
void ADC_Update(void)
{
	uint32_t ADC_VCC;
	int32_t ADC_TEMP;
	ADC_VCC = __LL_ADC_CALC_VREFANALOG_VOLTAGE(ADC_Read_Voltage(ADC1), LL_ADC_RESOLUTION_12B);
	MCU_VCC = (float)ADC_VCC / 1000;
	ADC_TEMP = __LL_ADC_CALC_TEMPERATURE(ADC_VCC, ADC_Read_Voltage(ADC1), LL_ADC_RESOLUTION_12B);
	SEGGER_RTT_printf(0, "VCC=%fV\r\n", MCU_VCC);
	SEGGER_RTT_printf(0, "Temp=%dC\r\n", ADC_TEMP);
}
				
__NO_RETURN void uart_transmit_thread(void *arg)
{
	osEventFlagsClear(uart_event_flagID, UART_RCV_DONE_FLAG);
  for (;;)
  {
		osEventFlagsWait(uart_event_flagID, UART_RCV_DONE_FLAG, osFlagsWaitAny, osWaitForever);
    memcpy(&uart_recv_cmd, uart_rcv_buf, uart_rcv_count);
    if (uart_recv_cmd.head != HEAD || uart_recv_cmd.tail != TAIL)
    {
      memset(uart_rcv_buf, 0, SIZE(uart_rcv_buf));
      uart_rcv_count = 0;
      uart_rcv_len = 0;
      continue;
    }
    else
    {
			SEGGER_RTT_printf(0, "uart recv:0x%x!\r\n", uart_cmd_type);
      switch (uart_recv_cmd.cmd)
      {	
				case GET_DATA_CMD:
					uart_rcv_flag = 1;
					PackAllData();
					UART2_Transmit8((uint8_t *)&uart_data, uart_data.data_len);
					break;
				case UPDATE_RTC_CMD:
					uart_rcv_flag = 1;
					break;
				default:
					break;
      }
      osEventFlagsClear(uart_event_flagID, UART_RCV_DONE_FLAG);
      memset(uart_rcv_buf, 0, SIZE(uart_rcv_buf));
      uart_state = RCV_HEAD;
      uart_rcv_count = 0;
      uart_rcv_len = 0;
    }
	}
}

uint16_t crc16(uint8_t *data, uint8_t len, uint16_t *table)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++)
  {
    crc = (crc << 8) ^ table[((crc >> 8) ^ data[i]) & 0xFF];
  }
  return crc;
}

void PackQueue(Queue* queue, sensor_data_struct* sensor_data)
{
	sensor_data->max = queue->max;
	sensor_data->min = queue->min;
	sensor_data->avg = queue->avg;
}

void PackAllData(void)
{
	PackQueue(&Voltage_queue, &uart_data.voltage);
	PackQueue(&Current_queue, &uart_data.current);
	PackQueue(&Power_queue, &uart_data.power);
	PackQueue(&Temperature_queue, &uart_data.temperature);
	uart_data.mAh = mAh;
	uart_data.mWh = mWh;
	uart_data.MCU_VCC = MCU_VCC;
	uart_data.crc = crc16((uint8_t*)&uart_data.data_len, uart_data.data_len - 4, (uint16_t*)table);
	uart_data.data_len = sizeof(uart_data) / sizeof(uint8_t);
}

void i2c_timer_cb(void *param)
{
	osKernelLock();
	INA226_Update();
	float tmp = TMP1075_ReadTemp();
	osKernelUnlock();
	time_past = (float)(osKernelGetTickCount() - i2c_last_tick) / 3600;
	i2c_last_tick = osKernelGetTickCount();
	osDelay(50);
	ADC_Update();
	enqueue(&Voltage_queue, ina226_info.Voltage);
	enqueue(&Current_queue, ina226_info.Current);
	enqueue(&Power_queue, ina226_info.Power);
	enqueue(&Temperature_queue, tmp);
	mAh += time_past * ina226_info.Current;
	mWh += time_past * ina226_info.Power;
	osDelay(1);
	//PackAllData();
}

void app_main(void *arg)
{
	uart_mutexId = osMutexNew(&uart_mutex_attr);
	uart_event_flagID = osEventFlagsNew(&FlagsAttr_uart_event);
	uart_transmit_ID = osThreadNew(uart_transmit_thread, NULL, &ThreadAttr_uart_transmit);
	osKernelLock();
	TMP1075_Init();
	INA226_Init();
	osKernelUnlock();
	timer0 = osTimerNew(&i2c_timer_cb, osTimerPeriodic, (void *)0, &timerAttr_soc_cb);
	osTimerStart(timer0, 1000);
	i2c_last_tick = osKernelGetTickCount();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  //NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	osKernelInitialize();
	app_main_ID = osThreadNew(app_main, NULL, &ThreadAttr_app_main);
	while (osKernelGetState() != osKernelReady)
	{
	};
	osKernelStart();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* LSI configuration and activation */
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1)
  {
  }

  LL_PWR_EnableBkUpAccess();
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the HSI */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(16000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(16000000);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */

   #define ADC_CHANNEL_CONF_RDY_TIMEOUT_MS ( 1U)
   #if (USE_TIMEOUT == 1)
   uint32_t Timeout ; /* Variable used for Timeout management */
   #endif /* USE_TIMEOUT */

  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_CONFIGURABLE);

   /* Poll for ADC channel configuration ready */
   #if (USE_TIMEOUT == 1)
   Timeout = ADC_CHANNEL_CONF_RDY_TIMEOUT_MS;
   #endif /* USE_TIMEOUT */
   while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0)
     {
   #if (USE_TIMEOUT == 1)
   /* Check Systick counter flag to decrement the time-out value */
   if (LL_SYSTICK_IsActiveCounterFlag())
     {
   if(Timeout-- == 0)
         {
   Error_Handler();
         }
     }
   #endif /* USE_TIMEOUT */
     }
   /* Clear flag ADC channel configuration ready */
	LL_ADC_ClearFlag_CCRDY(ADC1);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  LL_ADC_SetTriggerFrequencyMode(ADC1, LL_ADC_CLOCK_FREQ_MODE_HIGH);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_AWD_CH_VREFINT_REG|LL_ADC_AWD_CH_TEMPSENSOR_REG);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_160CYCLES_5);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_2, LL_ADC_SAMPLINGTIME_160CYCLES_5);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);

   /* Enable ADC internal voltage regulator */
   LL_ADC_EnableInternalRegulator(ADC1);
   /* Delay for ADC internal voltage regulator stabilization. */
   /* Compute number of CPU cycles to wait for, from delay in us. */
   /* Note: Variable divided by 2 to compensate partially */
   /* CPU processing cycles (depends on compilation optimization). */
   /* Note: If system core clock frequency is below 200kHz, wait time */
   /* is only a few CPU processing cycles. */
   uint32_t wait_loop_index;
   wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
   while(wait_loop_index != 0)
     {
   wait_loop_index--;
     }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_VREFINT);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_COMMON_1);
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_TEMPSENSOR);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_COMMON_1);

   /* Poll for ADC channel configuration ready */
   #if (USE_TIMEOUT == 1)
   Timeout = ADC_CHANNEL_CONF_RDY_TIMEOUT_MS;
   #endif /* USE_TIMEOUT */
   while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0)
     {
   #if (USE_TIMEOUT == 1)
   /* Check Systick counter flag to decrement the time-out value */
   if (LL_SYSTICK_IsActiveCounterFlag())
     {
   if(Timeout-- == 0)
         {
   Error_Handler();
         }
     }
   #endif /* USE_TIMEOUT */
     }
   /* Clear flag ADC channel configuration ready */
   LL_ADC_ClearFlag_CCRDY(ADC1);
  /* USER CODE BEGIN ADC1_Init 2 */
	LL_mDelay(1);
	LL_ADC_StartCalibration(ADC1);
	while (LL_ADC_IsCalibrationOnGoing(ADC1));
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  LL_RTC_InitTypeDef RTC_InitStruct = {0};

  if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSI)
  {
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  }

  /* Peripheral clock enable */
  LL_RCC_EnableRTC();
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_RTC);

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
  RTC_InitStruct.AsynchPrescaler = 127;
  RTC_InitStruct.SynchPrescaler = 255;
  LL_RTC_Init(RTC, &RTC_InitStruct);
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, 1);
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_SetTXRXSwap(USART2, LL_USART_TXRX_SWAPPED);
  LL_USART_ConfigAsyncMode(USART2);

  /* USER CODE BEGIN WKUPType USART2 */
	LL_USART_EnableIT_RXNE(USART2);
  /* USER CODE END WKUPType USART2 */

  LL_USART_Enable(USART2);

  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
