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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <motor.h>
#include <sensors.h>
//#include <inttypes.h>
#include <stdlib.h>

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim4_ch4;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for LineFollower_Ta */
osThreadId_t LineFollower_TaHandle;
const osThreadAttr_t LineFollower_Ta_attributes = {
  .name = "LineFollower_Ta",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 128 * 4
};
/* Definitions for RGB_Task */
osThreadId_t RGB_TaskHandle;
const osThreadAttr_t RGB_Task_attributes = {
  .name = "RGB_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for StatsPrint_Task */
osThreadId_t StatsPrint_TaskHandle;
const osThreadAttr_t StatsPrint_Task_attributes = {
  .name = "StatsPrint_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM15_Init(void);
void vLineFollowerTask(void *argument);
void vRGBLightTask(void *argument);
void vStatusPrintTask(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t distance_cm = 0;
volatile uint32_t pulse_ticks = 0;

IPSensor ips  = {0,0};
LTSensor sensor = {0,0,0,0};

extern volatile int datasentflag;

volatile uint32_t period_left = 0;
volatile uint32_t period_right = 0;

MotorHandle Mot_right = {
    .htim = &htim2,
    .channel_in1 = TIM_CHANNEL_1,   // PA0
    .channel_in2 = TIM_CHANNEL_2,   // PA1
    .ena_port = DCMot_right_ENB_GPO_GPIO_Port,
    .ena_pin = DCMot_right_ENB_GPO_Pin            // PC1
};

MotorHandle Mot_left = {
    .htim = &htim3,
    .channel_in1 = TIM_CHANNEL_1,   // PB4 (example, your mapping)
    .channel_in2 = TIM_CHANNEL_2,   // PB5 (example, your mapping)
    .ena_port = DCMot_left_ENA_GPO_GPIO_Port,
    .ena_pin = DCMot_left_ENA_GPO_Pin           // PA10
};

const uint8_t ramping_step = 5;

volatile uint8_t angle = 75;

#define PULSES_PER_REV 20

// Shared outputs (read in your task)
volatile uint32_t period_right_ticks = 0;  // TIM15 ticks between edges (right)
volatile uint32_t period_left_ticks  = 0;  // TIM15 ticks between edges (left)
volatile uint32_t last_right_ms = 0;
volatile uint32_t last_left_ms  = 0;

// Internals for overflow-safe measurement
volatile uint32_t tim15_overflow_count = 0;

uint32_t rpmR = 0;
uint32_t rpmL = 0;

uint8_t rxByte;              // one byte received
char rxLine[16];             // buffer for ASCII number
uint8_t rxIndex = 0;



void uart_receive_line(char *buf, uint16_t maxlen) {
    uint16_t i = 0;
    uint8_t c;
    while (i < maxlen - 1) {
        if (HAL_UART_Receive(&huart3, &c, 1, 5000) == HAL_OK) { // 5s timeout
            if (c == '\r' || c == '\n') break;
            buf[i++] = c;
        }
    }
    buf[i] = '\0';
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
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //xTraceInitialize();
  //xTraceEnable(TRC_START);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  MX_USART3_UART_Init();
  MX_TIM17_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  /*
  char rxbuf[64];

  // send "AT"
   uart_send("AT\r\n");
   uart_receive_line(rxbuf, sizeof rxbuf);
   // put a breakpoint here and check rxbuf, should be "OK"

   // set new baud
   uart_send("AT+UART=115200,0,0\r\n");
   uart_receive_line(rxbuf, sizeof rxbuf);
   // should reply "OK"
	*/

  //HAL_UART_Receive_DMA(&huart3, &rxByte, 1); // start DMA reception of 1 byte


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LineFollower_Ta */
  LineFollower_TaHandle = osThreadNew(vLineFollowerTask, NULL, &LineFollower_Ta_attributes);

  /* creation of RGB_Task */
  RGB_TaskHandle = osThreadNew(vRGBLightTask, NULL, &RGB_Task_attributes);

  /* creation of StatsPrint_Task */
  StatsPrint_TaskHandle = osThreadNew(vStatusPrintTask, NULL, &StatsPrint_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  	Set_LED(0, 255, 0, 0);
	Set_LED(1, 0, 255, 0);

	Set_LED(2, 0, 0, 255);
	Set_LED(3, 46, 89, 128);

	Set_LED(4, 156, 233, 100);
	Set_LED(5, 102, 0, 235);

	Set_LED(6, 47, 38, 77);
	Set_LED(7, 255, 200, 0);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 17;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 17;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 211;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 70;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 16;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 7;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 169;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1699;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 150;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 1699;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 7499;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMot_right_ENB_GPO_GPIO_Port, DCMot_right_ENB_GPO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMot_left_ENA_GPO_GPIO_Port, DCMot_left_ENA_GPO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DCMot_right_ENB_GPO_Pin */
  GPIO_InitStruct.Pin = DCMot_right_ENB_GPO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DCMot_right_ENB_GPO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IPS_right_GPI_Pin IPS_left_GPI_Pin LTS_left_GPI_Pin LTS_right_GPI_Pin
                           LTS_middle_GPI_Pin */
  GPIO_InitStruct.Pin = IPS_right_GPI_Pin|IPS_left_GPI_Pin|LTS_left_GPI_Pin|LTS_right_GPI_Pin
                          |LTS_middle_GPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMot_left_ENA_GPO_Pin */
  GPIO_InitStruct.Pin = DCMot_left_ENA_GPO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DCMot_left_ENA_GPO_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static uint32_t last_cap_right = 0, last_cap_left = 0;
	static uint32_t last_ovf_right = 0, last_ovf_left = 0;

	if (htim->Instance == TIM15) {

	        // Snapshot overflow counter first (prevents races)
	        uint32_t overflow_snapshot = tim15_overflow_count;

	        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {      // PA2 => Right
	            uint32_t capture_now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	            uint32_t ticks_since_last = (overflow_snapshot - last_ovf_right) * 65536u + (uint16_t)(capture_now - last_cap_right);

	            last_cap_right   = capture_now;
	            last_ovf_right   = overflow_snapshot;
	            period_right_ticks = ticks_since_last;
	            last_right_ms    = xTaskGetTickCountFromISR();
	        }
	        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {  // PA3 => Left
	            uint32_t capture_now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
	            uint32_t ticks_since_last =
	                (overflow_snapshot - last_ovf_left) * 65536u +
	                (uint16_t)(capture_now - last_cap_left);

	            last_cap_left    = capture_now;
	            last_ovf_left    = overflow_snapshot;
	            period_left_ticks = ticks_since_last;
	            last_left_ms     = xTaskGetTickCountFromISR();
	        }
	}


	static uint32_t echo_start = 0, echo_end = 0, edge_state = 0;

    if(htim->Instance == TIM8 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  {

        if(edge_state == 0)  { // Rising edge

            echo_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
            edge_state = 1;

        } else { // Falling edge

            echo_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            edge_state = 0;

            if (echo_end >= echo_start) {
            	pulse_ticks = (echo_end - echo_start);
            } else {
            	pulse_ticks = (65536 - echo_start + echo_end);
            }
        }
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_4);
	datasentflag=1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(StatsPrint_TaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*
// For Bluetooth recieving

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        char c = (char)rxByte;

        if (c == '\r') {
            // ignore CR
        }
        else if (c == '\n') {
            // end of line → convert string to number
            rxLine[rxIndex] = '\0';
            int v = atoi(rxLine);
            if (v < 0) v = 0;
            if (v > 180) v = 180;
            angle = (uint8_t)v;

            rxIndex = 0; // reset for next number
        }
        else {
            if (rxIndex < sizeof(rxLine) - 1) {
                rxLine[rxIndex++] = c;
            } else {
                rxIndex = 0; // overflow → reset
            }
        }

        HAL_UART_Receive_DMA(&huart3, &rxByte, 1);
    }
}
 */

void to_binary(char *dest, uint8_t val, uint8_t bits)
{
    for (int i = bits - 1; i >= 0; --i) {
        *dest++ = (val & (1 << i)) ? '1' : '0';
    }
    *dest = '\0';
}

int8_t ForwardSpeed = 50;
int8_t ForwardCurveSpeed = 50;
int8_t stop = 0;
uint8_t min_dist = 5;

/* USER CODE END 4 */

/* USER CODE BEGIN Header_vLineFollowerTask */
/**
  * @brief  Function implementing the LineFollower_Ta thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_vLineFollowerTask */
void vLineFollowerTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	//const TickType_t xPeriod_ramp = pdMS_TO_TICKS(1);
	const TickType_t xPeriod = pdMS_TO_TICKS(1);
	TickType_t xLastWakeTime = xTaskGetTickCount();

	Motors_Init(&Mot_right, &Mot_left);


	/*
	HAL_GPIO_WritePin(DCMot_right_ENB_GPO_GPIO_Port, DCMot_right_ENB_GPO_Pin , GPIO_PIN_SET);
	HAL_GPIO_WritePin(DCMot_left_ENA_GPO_GPIO_Port, DCMot_left_ENA_GPO_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	*/

    USS_Init(&htim17, TIM_CHANNEL_1, &htim8, TIM_CHANNEL_1);
    LTS_Config cfg = {GPIOC, LTS_left_GPI_Pin, GPIOC, LTS_middle_GPI_Pin, GPIOC, LTS_right_GPI_Pin};
    IPS_Config ips_cfg = {GPIOC, IPS_left_GPI_Pin,  GPIOC, IPS_right_GPI_Pin};

    TickType_t noLineStartTime = 0;
    //bool noStartActive = false;
    bool noLineActive = false;

	vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(1000));
  /* Infinite loop */
	for(;;) {
	  distance_cm = (uint32_t)USS_get_value(pulse_ticks);
	  sensor = get_LTS_value(&cfg);
	  ips = get_IPS_value(&ips_cfg);

	  /*
	  if (distance_cm <= min_dist){
		  stop_robot();
		  continue;
	  }
	  */
	  if (sensor.current_state == NO_LINE ) {
		  if (!noLineActive){
			  noLineStartTime = xTaskGetTickCount();
			  noLineActive = true;
		  } else {
			  if((xTaskGetTickCount() - noLineStartTime) >= pdMS_TO_TICKS(1000)){
				  stop_robot();
				  vTaskDelay(pdMS_TO_TICKS(10));
				  continue;
			  }
		  }
	  } else {
		  noLineActive = false;
	  }

	  switch(sensor.current_state){
	  case NO_LINE: 	move_robot(ForwardSpeed, ForwardSpeed); break;
	  case MIDDLE:		move_robot(ForwardSpeed, ForwardSpeed); break;

	  case RIGHT:		move_robot(ForwardCurveSpeed,stop); break;
	  //case RIGHT_CURVE:	move_robot(ForwardSpeed,curveSpeed ); break;

	  case LEFT:		move_robot(stop, ForwardCurveSpeed); break;
	  //case LEFT_CURVE:	move_robot(curveSpeed,ForwardSpeed); break;

	  case U_TURN:		stop_robot(); 	break;
	  case JUNCTION:	stop_robot(); 	break;
	  }


	  vTaskDelayUntil(&xLastWakeTime, xPeriod);


	  /*
	  //move_forward
	  Motor_SetPWM(&Mot_right, MOTOR_DIR_FORWARD, 50.0, ramping_step);
	  Motor_SetPWM(&Mot_left, MOTOR_DIR_FORWARD, 50.0, ramping_step);
	  vTaskDelayUntil(&xLastWakeTime, xPeriod);
	  */

/*
	  Motor_SetPWM(&Mot_right, MOTOR_DIR_REVERSE, 50.0, ramping_step);
	  Motor_SetPWM(&Mot_left, MOTOR_DIR_REVERSE, 50.0, ramping_step);
	  vTaskDelayUntil(&xLastWakeTime, xPeriod);

	  Motor_SetPWM(&Mot_right, MOTOR_DIR_COAST, 0.0, ramping_step);
	  Motor_SetPWM(&Mot_left, MOTOR_DIR_COAST, 0.0, ramping_step);
	  vTaskDelayUntil(&xLastWakeTime, xPeriod);

*/	}

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vRGBLightTask */
/**
* @brief Function implementing the RGB_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vRGBLightTask */
void vRGBLightTask(void *argument)
{
  /* USER CODE BEGIN vRGBLightTask */
	const TickType_t xPeriod = pdMS_TO_TICKS(500);
	TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  for (int i=0; i<46; i++)
	  {
		  Set_Brightness(i);
		  WS2812_Send();
		  vTaskDelayUntil( &xLastWakeTime, xPeriod);
	  }

	  for (int i=45; i>=0; i--)
	  {
		  Set_Brightness(i);
		  WS2812_Send();
		  vTaskDelayUntil( &xLastWakeTime, xPeriod);
	  }
  }
  /* USER CODE END vRGBLightTask */
}

/* USER CODE BEGIN Header_vStatusPrintTask */
/**
* @brief Function implementing the StatsPrint_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vStatusPrintTask */
void vStatusPrintTask(void *argument)
{
  /* USER CODE BEGIN vStatusPrintTask */
	//static char txBuffer[90];
	//static char binObstacles[3 + 1]; // 2 bits + null terminator
	//static char binLines[4 + 1];     // 3 bits + null terminator
	const TickType_t xPeriod = pdMS_TO_TICKS(5000);
	TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  /*
	  uint8_t obstVal = (ips.leftObstacle_detected << 1) | ips.rightObstacle_detected;
	  uint8_t linesVal = (sensor.leftLine_detected << 2) | (sensor.middleLine_detected << 1) | sensor.rightLine_detected;


	  to_binary(binObstacles,  obstVal,  2);
	  to_binary(binLines, linesVal, 3);
		*/
	  /*
	  int len = snprintf(txBuffer, sizeof(txBuffer),
		  //"%u    [cm]    \r\n",
		 "%lu [cm], %s [LR], %s [LMR], %u [deg], %lu [L cm/s], %lu [R cm/s]\r\n",
		  distance_cm, binObstacles, binLines,  angle, 	  rpmL,   rpmR 		) + 4;
		*/
	  //HAL_UART_Transmit_DMA(&huart3, (uint8_t *)txBuffer, len);

	  // Block until DMA finishes
	  vTaskDelayUntil(&xLastWakeTime, xPeriod);
	  //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
  /* USER CODE END vStatusPrintTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM15)
  {
	  tim15_overflow_count++;
  }
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
