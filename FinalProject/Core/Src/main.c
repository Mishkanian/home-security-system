/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISARMED 0
#define ARMED 1
#define ALARM 2

#define NOTHING_HAPPENED 0
#define BUTTON_TOGGLED 1
#define SHAKE_DETECTED 2
#define EVERYTHING_IS_OK 3
#define TIMEOUT 4

#define ALARM_LED_GPIO_Port LED3_WIFI__LED4_BLE_GPIO_Port
#define ALARM_LED_Pin LED3_WIFI__LED4_BLE_Pin
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
//#if defined (TERMINAL_USE)
// UART_HandleTypeDef huart1;
//#endif /* TERMINAL_USE */
static uint8_t http[1024];
static uint8_t IP_Addr[4];
static int LedState = 0;
static int AlarmState = DISARMED;
static int16_t accelXYZ[3] = {0};
osMessageQueueId_t evtQueueHandle;
osSemaphoreId_t buttonSemHandle;
osTimerId_t alarmBlinkTimerHandle;
osTimerId_t codeTimeoutTimerHandle;
osThreadId_t stateTaskHandle;
osThreadId_t sensorTaskHandle;
osThreadId_t buttonTaskHandle;
volatile int waitingForCode = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void SensorTask(void *argument);
void StateTask(void *argument);
void ButtonTask(void *argument);
void AlarmBlinkTimerCallback(void *argument);
void CodeTimeoutCallback(void *argument);
static void UpdateLedsForState(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Update SSID and PASSWORD with own Access point settings */
#define SSID "MJM iPhone"
#define PASSWORD "wow123wifi"
#define PORT 80
#define TERMINAL_USE
#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT 10000
#define SOCKET 0
#ifdef TERMINAL_USE
#define LOG(a) printf a
#else
#define LOG(a)
#endif

#if defined (TERMINAL_USE)
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif /* TERMINAL_USE */
//static void SystemClock_Config(void);
static WIFI_Status_t SendWebPage(uint8_t ledIsOn);
static int wifi_server(void);
static int wifi_start(void);
static int wifi_connect(void);
static bool WebServerProcess(void);
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  BSP_ACCELERO_Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  buttonSemHandle = osSemaphoreNew(1, 0, NULL);

  if (buttonSemHandle == NULL) {
      uint8_t msg[] = "buttonSemHandle NULL\r\n";
      HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, 100);
  }

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  alarmBlinkTimerHandle = osTimerNew(AlarmBlinkTimerCallback, osTimerPeriodic, NULL, NULL);
  codeTimeoutTimerHandle = osTimerNew(CodeTimeoutCallback, osTimerOnce, NULL, NULL);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  evtQueueHandle = osMessageQueueNew(8, sizeof(uint8_t), NULL);

  if (evtQueueHandle == NULL) {
      uint8_t msg[] = "evtQueueHandle NULL\r\n";
      HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, 100);
  }

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE BEGIN RTOS_THREADS */
  const osThreadAttr_t stateTask_attributes = {
    .name = "stateTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal,
  };
  stateTaskHandle = osThreadNew(StateTask, NULL, &stateTask_attributes);
  if (stateTaskHandle == NULL) {
      uint8_t msg[] = "stateTask create FAILED\r\n";
      HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, 100);
  }

  const osThreadAttr_t sensorTask_attributes = {
    .name = "sensorTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityBelowNormal,
  };
  sensorTaskHandle = osThreadNew(SensorTask, NULL, &sensorTask_attributes);
  if (sensorTaskHandle == NULL) {
      uint8_t msg[] = "sensorTask create FAILED\r\n";
      HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, 100);
  }

  const osThreadAttr_t buttonTask_attributes = {
    .name = "buttonTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal,
  };
  buttonTaskHandle = osThreadNew(ButtonTask, NULL, &buttonTask_attributes);
  if (buttonTaskHandle == NULL) {
      uint8_t msg[] = "buttonTask create FAILED\r\n";
      HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, 100);
  }


  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000E14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void UpdateLedsForState(void)
{
  // DISARMED:led off
  // ARMED:led on
  // ALARM:blink

  if (AlarmState == DISARMED) {
    osTimerStop(alarmBlinkTimerHandle);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ALARM_LED_GPIO_Port, ALARM_LED_Pin, GPIO_PIN_RESET);
  }
  else if (AlarmState == ARMED) {
    osTimerStop(alarmBlinkTimerHandle);
    HAL_GPIO_WritePin(ALARM_LED_GPIO_Port, ALARM_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  }
  else if (AlarmState == ALARM) {
    // blink quickly
	osTimerStop(alarmBlinkTimerHandle);
    osTimerStart(alarmBlinkTimerHandle, 100);
  }
}

void AlarmBlinkTimerCallback(void *argument)
{
  //if (AlarmState == ALARM) {
    HAL_GPIO_TogglePin(ALARM_LED_GPIO_Port, ALARM_LED_Pin);
  //}
}

void CodeTimeoutCallback(void *argument)
{
  uint8_t event = TIMEOUT;
  osMessageQueuePut(evtQueueHandle, &event, 0, 0);
}

void ButtonTask(void *argument)
{
  uint8_t event;
  for (;;)
  {
    osSemaphoreAcquire(buttonSemHandle, osWaitForever);
	uint8_t msg[] = "Button pressed.\r\n";
	HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, 100);
	// delay to prevent debouncing issue
    osDelay(50);
    event = BUTTON_TOGGLED;
    osMessageQueuePut(evtQueueHandle, &event, 0, 0);
  }
}


void SensorTask(void *argument)
{
  uint8_t event;
  const int SHAKE_THRESHOLD = 1100;
  for (;;)
  {
    BSP_ACCELERO_AccGetXYZ(accelXYZ);

    int16_t ax = accelXYZ[0];
    int16_t ay = accelXYZ[1];
    int16_t az = accelXYZ[2];

    if (AlarmState == ARMED) {
      if (ax > SHAKE_THRESHOLD || ax < -SHAKE_THRESHOLD ||
          ay > SHAKE_THRESHOLD || ay < -SHAKE_THRESHOLD ||
          az > SHAKE_THRESHOLD || az < -SHAKE_THRESHOLD)
      {
    	event = SHAKE_DETECTED;
        osMessageQueuePut(evtQueueHandle, &event, 0, 0);
      }
    }
    osDelay(100);
  }
}


void StateTask(void *argument)
{
  uint8_t event;
  uint8_t hello2[] = "StateTask started\r\n";
  HAL_UART_Transmit(&huart1, hello2, sizeof(hello2)-1, 100);
  uint8_t msg[50];
  // start disarmed
  AlarmState = DISARMED;
  UpdateLedsForState();
  uint8_t hello[] = "System is DISARMED.\r\n";
  HAL_UART_Transmit(&huart1, hello, sizeof(hello) - 1, 100);

  for (;;)
  {
    if (osMessageQueueGet(evtQueueHandle, &event, NULL, osWaitForever) == osOK)
    {
      if (AlarmState == DISARMED)
      {
        if (event == BUTTON_TOGGLED) {
        	AlarmState = ARMED;
        	snprintf((char *)msg, sizeof(msg), "System is ARMED.\r\n");
        	HAL_UART_Transmit(&huart1, msg, strlen((char *)msg), 100);
        }
      }
      else if (AlarmState == ARMED)
      {
    	    if (event == BUTTON_TOGGLED && !waitingForCode) {
    	        AlarmState = DISARMED;
    	        snprintf((char *)msg, sizeof(msg), "System is DISARMED.\r\n");
    	        HAL_UART_Transmit(&huart1, msg, strlen((char *)msg), 100);
    	    }
    	    else if (event == SHAKE_DETECTED && !waitingForCode) {
    	        // sensor triggered, ARMED, start 5sec countdown
    	    	waitingForCode = 1;
    	        osTimerStart(codeTimeoutTimerHandle, 5000); // 5 seconds
    	        snprintf((char *)msg, sizeof(msg),"Motion detected! Enter code within 5 seconds.\r\n");
    	        HAL_UART_Transmit(&huart1, msg, strlen((char *)msg), 100);
    	        // NOTE: AlarmState stays ARMED here
    	    }
    	    else if (event == EVERYTHING_IS_OK && waitingForCode) {
    	        // correct code entered in time
    	    	waitingForCode = 0;
    	        osTimerStop(codeTimeoutTimerHandle);
    	        snprintf((char *)msg, sizeof(msg), "Code accepted, alarm canceled. System is ARMED.\r\n");
    	        HAL_UART_Transmit(&huart1, msg, strlen((char *)msg), 100);
    	        // AlarmState remains ARMED
    	    }
    	    else if (event == TIMEOUT && waitingForCode) {
    	    	waitingForCode = 0;
    	        AlarmState = ALARM;
    	        snprintf((char *)msg, sizeof(msg), "CODE TIMEOUT, ALARM ACTIVATED.\r\n");
    	        HAL_UART_Transmit(&huart1, msg, strlen((char *)msg), 100);
    	    }
      }
      else if (AlarmState == ALARM)
      {
        if (event == BUTTON_TOGGLED || event == EVERYTHING_IS_OK) {
        	AlarmState = DISARMED;
        	snprintf((char *)msg, sizeof(msg), "System is DISARMED.\r\n");
        	HAL_UART_Transmit(&huart1, msg, strlen((char *)msg), 100);
        }
      }
      // update LED
      UpdateLedsForState();
    }
  }
}



static int wifi_start(void)
{
	uint8_t MAC_Addr[6];
	/*Initialize and use WIFI module */
	if(WIFI_Init() == WIFI_STATUS_OK)
	{
		printf("ES-WIFI Initialized.\r\n");
		if(WIFI_GetMAC_Address(MAC_Addr, sizeof(MAC_Addr)) ==
		WIFI_STATUS_OK)
		{
			printf("> eS-WiFi module MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\r\n",
			MAC_Addr[0],
			MAC_Addr[1],
			MAC_Addr[2],
			MAC_Addr[3],
			MAC_Addr[4],
			MAC_Addr[5]);
		}
	else
	{
		LOG(("> ERROR : CANNOT get MAC address\r\n"));
		return -1;
	}
	}
	else
	{
		return -1;
	}
	return 0;
}

int wifi_connect(void)
{
	wifi_start();
	printf("\nConnecting to %s , %s\r\n",SSID,PASSWORD);
	if( WIFI_Connect(SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK)
	{
		if(WIFI_GetIP_Address(IP_Addr, sizeof(IP_Addr)) == WIFI_STATUS_OK)
		{
			LOG(("> es-wifi module connected: got IP Address : %d.%d.%d.%d\r\n",
			IP_Addr[0],
			IP_Addr[1],
			IP_Addr[2],
			IP_Addr[3]));
			int len = 100;
			uint8_t buffer[len];
			snprintf(buffer, sizeof(buffer), "es-wifi module connected: got IP Address : %d.%d.%d.%d\r\n", IP_Addr[0],
			IP_Addr[1],
			IP_Addr[2],
			IP_Addr[3] );
			buffer[len-1] = '\0';
			HAL_UART_Transmit(&huart1, buffer, strlen(buffer), 100);
		}
		else
		{
			LOG((" ERROR : es-wifi module CANNOT get IP address\r\n"));
			return -1;
		}
	}
	else
	{
		LOG(("ERROR : es-wifi module NOT connected\r\n"));
		return -1;
	}
	return 0;
}

int wifi_server(void)
{
	bool StopServer = false;
	LOG(("\nRunning HTML Server test\r\n"));
	if (wifi_connect()!=0) return -1;
	if (WIFI_STATUS_OK!=WIFI_StartServer(SOCKET, WIFI_TCP_PROTOCOL, 1,"", PORT))
	{
		LOG(("ERROR: Cannot start server.\r\n"));
	}
	LOG(("Server is running and waiting for an HTTP Client connection to %d.%d.%d.%d\r\n",IP_Addr[0],IP_Addr[1],IP_Addr[2],IP_Addr[3]));
	do
	{
	uint8_t RemoteIP[4];
	uint16_t RemotePort;
	while (WIFI_STATUS_OK !=
	WIFI_WaitServerConnection(SOCKET,1000,RemoteIP,sizeof(RemoteIP),&RemotePort))
	{
		//LOG(("Waiting connection to %d.%d.%d.%d\r\n",IP_Addr[0],IP_Addr[1],IP_Addr[2],IP_Addr[3]));
		osDelay(10); // yield
	}
	LOG(("Client connected %d.%d.%d.%d:%d\r\n",RemoteIP[0],RemoteIP[1],RemoteIP[2],RemoteIP[3],RemotePort));
	StopServer=WebServerProcess();
	if(WIFI_CloseServerConnection(SOCKET) != WIFI_STATUS_OK)
	{
		LOG(("ERROR: failed to close current Server connection\r\n"));
	return -1;
	}
	}
	while(StopServer == false);
	if (WIFI_STATUS_OK!=WIFI_StopServer(SOCKET))
	{
		LOG(("ERROR: Cannot stop server.\r\n"));
	}
	LOG(("Server is stop\r\n"));
	return 0;
}


static bool WebServerProcess(void)
{
	//uint8_t temp;
	uint16_t respLen;
	static uint8_t resp[1024];
	bool stopserver=false;
	if (WIFI_STATUS_OK == WIFI_ReceiveData(SOCKET, resp, 1000, &respLen,WIFI_READ_TIMEOUT))
	{
		//LOG(("get %d byte from server\r\n",respLen));
	if( respLen > 0)
	{
		if(strstr((char *)resp, "GET")) /* GET: put web page */
		{
			if(SendWebPage(LedState) != WIFI_STATUS_OK)
			{
				LOG(("> ERROR : Cannot send web page\n\r"));
			}
//			else
//			{
//				LOG(("Send page after GET command\r\n"));
//			}
		}
		else if(strstr((char *)resp, "POST"))/* POST: received info */
		{
			//LOG(("Post request\n"));
//			if(strstr((char *)resp, "radio"))
//			{
//				if(strstr((char *)resp, "radio=0"))
//				{
//					LedState = 0;
//					// BSP_LED_Off(LED2);
//					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
//				}
//				else if(strstr((char *)resp, "radio=1"))
//				{
//					LedState = 1;
//					//BSP_LED_On(LED2);
//					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
//				}
//
//			}
			if (strstr((char *)resp, "code_ok=1"))
			{
				//LOG(("Code OK received from web client\n"));
				if (waitingForCode || AlarmState == ALARM)
				{
					uint8_t evt = EVERYTHING_IS_OK;
					osMessageQueuePut(evtQueueHandle, &evt, 0, 0);
				}
			}
			if(strstr((char *)resp, "stop_server"))
			{
				if(strstr((char *)resp, "stop_server=0"))
				{
					stopserver = false;
				}
				else if(strstr((char *)resp, "stop_server=1"))
				{
					stopserver = true;
				}
			}
		if (SendWebPage(LedState) != WIFI_STATUS_OK)
		 {
		        LOG(("> ERROR : Cannot send web page after POST\n"));
		    }
//		else
//		{
//			LOG(("Send Page after POST command\n"));
//		}
		}
	}
}
	else
	{
		LOG(("Client close connection\n"));
	}
	osDelay(1); //yield
	return stopserver;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) {
	    case GPIO_PIN_1:
	      // wifi interrupt
	      SPI_WIFI_ISR();
	      break;

	    case BUTTON_EXTI13_Pin:
	      // button
	      osSemaphoreRelease(buttonSemHandle);
	      break;

	    default:
	      break;
	}
}
void SPI3_IRQHandler(void)
{
/* USER CODE BEGIN SPI3_IRQn 0 */
//
/* USER CODE END SPI3_IRQn 0 */
HAL_SPI_IRQHandler(&hspi);
/* USER CODE BEGIN SPI3_IRQn 1 */
//
/* USER CODE END SPI3_IRQn 1 */
}

/**
* @brief Send HTML page
* @param None
* @retval None
*/
static WIFI_Status_t SendWebPage(uint8_t ledIsOn)
{
	//uint8_t temp[50];
	uint16_t SentDataLength;
	WIFI_Status_t ret;
	/* construct web page content */
	strcpy((char *)http, (char *)"HTTP/1.0 200 OK\r\nContent-Type:text/html\r\nPragma: no-cache\r\n\r\n");
	strcat((char *)http, (char *)"<html>\r\n<body>\r\n");
	strcat((char *)http, (char *)"<title>STM32 Web Server</title>\r\n");
	strcat((char *)http, (char *)"<h2>Home Security System</h2>\r\n");
	strcat((char *)http, (char *)"<br /><hr>\r\n");
	//strcat((char *)http, (char *)"<p><form method=\"POST\"><strong>Temp:<input type=\"text\" value=\"");
	//sprintf((char *)temp, "%d", temperature);
	//strcat((char *)http, (char *)temp);
	//strcat((char *)http, (char *)"\"> <sup>O</sup>C");
	strcat((char *)http, "<p><strong>System state: ");
	if (AlarmState == DISARMED) {
		strcat((char *)http, "DISARMED");
	}
	else if (AlarmState == ARMED){
		strcat((char *)http, "ARMED");
	}
	else if (AlarmState == ALARM) {
		strcat((char *)http, "ALARM");
	}
	strcat((char *)http, "</strong></p>\r\n");
//	if (ledIsOn)
//	{
//	strcat((char *)http, (char *)"<p><input type=\"radio\"name=\"radio\" value=\"0\" >RSG LED off");
//	strcat((char *)http, (char *)"<br><input type=\"radio\"name=\"radio\" value=\"1\" checked>RSG LED on");
//	}
//	else
//	{
//	strcat((char *)http, (char *)"<p><input type=\"radio\"name=\"radio\" value=\"0\" checked>RSG LED off");
//	strcat((char *)http, (char *)"<br><input type=\"radio\"name=\"radio\" value=\"1\" >RSG LED on");
//	}

    //strcat((char *)http, "</strong><p><input type=\"submit\" value=\"Set LED\"></form>");
    // enter code to cancel alarm
    strcat((char *)http, "<p><form method=\"POST\">");
    strcat((char *)http, "<input type=\"hidden\" name=\"code_ok\" value=\"1\">");
    strcat((char *)http, "<input type=\"submit\" value=\"Enter Code\">");
    //strcat((char *)http, (char *)"</strong><p><input type=\"submit\"></form></span>");
    strcat((char *)http, (char *)"</body>\r\n</html>\r\n");

	ret = WIFI_SendData(0, (uint8_t *)http, strlen((char *)http), &SentDataLength, WIFI_WRITE_TIMEOUT);
	if((ret == WIFI_STATUS_OK) && (SentDataLength != strlen((char*)http)))
	{
		ret = WIFI_STATUS_ERROR;
	}
	return ret;
}

#if defined (TERMINAL_USE)
/**
* @brief Retargets the C library printf function to the USART.
* @param None
* @retval None
*/
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
return ch;
}
#endif /* TERMINAL_USE */

#ifdef USE_FULL_ASSERT
/**
* @brief Reports the name of the source file and the source line
number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{


}
#endif



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  wifi_server();
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
