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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for LineFollower_Ta */
osThreadId_t LineFollower_TaHandle;
const osThreadAttr_t LineFollower_Ta_attributes = {
  .name = "LineFollower_Ta",
  .priority = (osPriority_t) osPriorityRealtime7,
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
/* Definitions for gEncMtx */
osSemaphoreId_t gEncMtxHandle;
const osSemaphoreAttr_t gEncMtx_attributes = {
  .name = "gEncMtx"
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM16_Init(void);
void vLineFollowerTask(void *argument);
void vRGBLightTask(void *argument);
void vStatusPrintTask(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t pulse_ticks = 0;
const uint8_t min_dist = 5;
const uint8_t uss_offset = 3;

#define VOLTAGE_LEVEL 95   // 95 = 9.5V, 80 = 8.0V, etc.

#if VOLTAGE_LEVEL == 95
    #define R_PWM        51
    #define L_PWM        50
    #define R_PWM_Curve  58
    #define L_PWM_Curve  57
#elif VOLTAGE_LEVEL == 80
    #define R_PWM        61
    #define L_PWM        60
    #define R_PWM_Curve  61
    #define L_PWM_Curve  60
#endif

// Now you can declare variables using these constants
uint16_t R_PWM_Value       = R_PWM;
uint16_t L_PWM_Value       = L_PWM;
uint16_t R_PWM_Curve_Value = R_PWM_Curve;
uint16_t L_PWM_Curve_Value = L_PWM_Curve;


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
  MX_TIM2_Init();
  MX_TIM17_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_TIM16_Init();
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

  /* Create the semaphores(s) */
  /* creation of gEncMtx */
  gEncMtxHandle = osSemaphoreNew(1, 1, &gEncMtx_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  //gEncMtx = xSemaphoreCreateMutex();
  //configASSERT(gEncMtx != NULL);
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
  /*
  	Set_LED(0, 255, 0, 0);
	Set_LED(1, 0, 255, 0);

	Set_LED(2, 0, 0, 255);
	Set_LED(3, 46, 89, 128);

	Set_LED(4, 156, 233, 100);
	Set_LED(5, 102, 0, 235);

	Set_LED(6, 47, 38, 77);
	Set_LED(7, 255, 200, 0);

	Set_Brightness(45);
	//WS2812_Send();
	*/

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 6;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 16;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 42499;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 3999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  sConfigOC.Pulse = 0;
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

  /*Configure GPIO pins : SS_right_T15C1_ICDM_Pin SS_left_T15C2_ICDM_Pin */
  GPIO_InitStruct.Pin = SS_right_T15C1_ICDM_Pin|SS_left_T15C2_ICDM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMot_left_ENA_GPO_Pin */
  GPIO_InitStruct.Pin = DCMot_left_ENA_GPO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DCMot_left_ENA_GPO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LTS_left_GPI_Pin LTS_right_GPI_Pin LTS_middle_GPI_Pin */
  GPIO_InitStruct.Pin = LTS_left_GPI_Pin|LTS_right_GPI_Pin|LTS_middle_GPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RGB_Lights_T4C4_PWM_Pin */
  GPIO_InitStruct.Pin = RGB_Lights_T4C4_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(RGB_Lights_T4C4_PWM_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	/*
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

*/

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
	motors_init(&Mot_right, &Mot_left);
	USS_init(&htim17, TIM_CHANNEL_1, &htim8, TIM_CHANNEL_1);
  /* Infinite loop */
  for(;;)
  {
	  uint32_t uss_d = USS_get_value(pulse_ticks);
	  if(uss_d > 9){
		uint8_t state = ((GPIOC->IDR & GPIO_PIN_10) >> 9)| ((GPIOC->IDR & GPIO_PIN_11) >> 11);
		switch (state) {
			case 0b11: stop_robot(); 				break; 	// both
			case 0b10: move_robot(0,R_PWM_Curve); 	break; 	// left
			case 0b01: move_robot(L_PWM_Curve,0); 	break; 	// right
			default:   move_robot(L_PWM, R_PWM);  	break;	// none
		}
	  } else {
			stop_robot();
		}
  }
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
	//const TickType_t xPeriod = pdMS_TO_TICKS(500);
	TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  /*
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
	  */
	  vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(40000));

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
	/*
    //static char txBuffer[128];
	//static char binObstacles[3 + 1]; // 2 bits + null terminator
	//static char binLines[4 + 1];     // 3 bits + null terminator

   // HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);


	EncoderConfig cfg1 = {
		.htim = &htim1,
		.inverted = false,
		.cpr = 3840u,           // counts per revolution
		.wheel_circum_m = 0.210f // circumference in meters
	};

	Encoder_Init(&enc1, &cfg1);
*/

	// Test send over UART
	const char msg[] = "Encoder test start\r\n";
	HAL_UART_Transmit(&huart3, (uint8_t*)msg, sizeof(msg)-1, HAL_MAX_DELAY);

	TickType_t xLastWake = xTaskGetTickCount();
	vTaskDelayUntil(&xLastWake,  pdMS_TO_TICKS(1000));

  /* Infinite loop */
  for(;;)
  {
	  /*
	  // TEST MESSAGE
	  // Build the message (could include encoder/sensor values)
	  int len = snprintf(txBuffer, sizeof(txBuffer), "Hello from FreeRTOS @ %lu ms\r\n",
						 (unsigned long)xTaskGetTickCount() * portTICK_PERIOD_MS);

	  if (len > 0)
	  {
		  // Non-blocking send via DMA
		  HAL_UART_Transmit_DMA(&huart3, (uint8_t *)txBuffer, len);

		  // Optionally wait until DMA finishes before sending again
		  // ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
	  }



	  //Encoder_Update(&enc1, 0.1f); // dt = 0.1 s

	  //int32_t counts = Encoder_GetCounts(&enc1);
	  //float rpm = Encoder_GetRPM(&enc1);
	  //long unsigned int mps = Encoder_GetLinearSpeed(&enc1);

	  //int len = snprintf(txBuffer, sizeof(txBuffer), "C:%ld RPM:%.2f m/s:%lu\r\n",
		//				 (long)counts, rpm, mps);

	  int len = snprintf(txBuffer, sizeof(txBuffer), "p:%lu m | v:%lu m/s\r\n",
						 encoder_position, encoder_velocity);

	  if (len > 0 && len < sizeof(txBuffer)) {
	  	  HAL_UART_Transmit_DMA(&huart3, (uint8_t *)txBuffer, len);
	  } else {

  		  // snprintf failed (len < 0) or message truncated (len >= sizeof(txBuffer))
  		  dropCount++;

  		  // Optional: send a fallback warning (short and safe)
  		  const char warn[] = "TX DROP\r\n";
  		  HAL_UART_Transmit_DMA(&huart3, (uint8_t *)warn, sizeof(warn) - 1);
  		  continue;
  	  }


      // ---- Snapshot shared data ----
	  EncoderData_t s;
	  s.L_counts = Encoder_GetCounts(&leftEnc);
	  s.L_rpm    = Encoder_GetRPM(&leftEnc);
	  s.L_mps    = Encoder_GetLinearSpeed(&leftEnc);

	  //s.R_counts = Encoder_GetCounts(&rightEnc);
	  //s.R_rpm    = Encoder_GetRPM(&rightEnc);
	  //s.R_mps    = Encoder_GetLinearSpeed(&rightEnc);




	  if (gEncMtx) {
		  xSemaphoreTake(gEncMtx, portMAX_DELAY);
		  s = gEncData;
		  xSemaphoreGive(gEncMtx);
	  } else {
		  s = gEncData;
	  }
	  s = gEncData;


	  // ---- Format your payload (fit within txBuffer) ----
	  // Keep it short to avoid heap/stack bloat; floats are OK if task stack is big enough.
	  int len = snprintf(txBuffer, sizeof(txBuffer),
						 "L:%ld %.2frpm %.3fm/s \r\n",
						 (long)s.L_counts, s.L_rpm, s.L_mps);

	  if (len > 0 && len < sizeof(txBuffer)) {

		  // Non-blocking send via DMA
		  HAL_UART_Transmit_DMA(&huart3, (uint8_t *)txBuffer, len);

	  } else {

		  // snprintf failed (len < 0) or message truncated (len >= sizeof(txBuffer))
		  dropCount++;

		  // Optional: send a fallback warning (short and safe)
		  const char warn[] = "TX DROP\r\n";
		  HAL_UART_Transmit_DMA(&huart3, (uint8_t *)warn, sizeof(warn) - 1);
		  continue;

	  }



	  uint8_t obstVal = (ips.leftObstacle_detected << 1) | ips.rightObstacle_detected;
	  uint8_t linesVal = (sensor.leftLine_detected << 2) | (sensor.middleLine_detected << 1) | sensor.rightLine_detected;

	  to_binary(binObstacles,  obstVal,  2);
	  to_binary(binLines, linesVal, 3);

	  int len = snprintf(txBuffer, sizeof(txBuffer),
		  //"%u    [cm]    \r\n",
		 "%lu [cm], %s [LR], %s [LMR], %u [deg], %lu [L cm/s], %lu [R cm/s]\r\n",
		  distance_cm, binObstacles, binLines,  angle, 	  rpmL,   rpmR 		) + 4;


	  // else: previous transfer still running → drop this frame (or add a queue)
	  //ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000));


	   */
	  vTaskDelayUntil( &xLastWake, pdMS_TO_TICKS(40000));

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
	/*
	timer_counter = __HAL_TIM_GET_COUNTER(&htim1);
  // measure velocity, position
  update_encoder(&enc_instance_mot, &htim1);
  //encoder_position = enc_instance_mot.position;
  //encoder_velocity = enc_instance_mot.velocity;
  static char txBuffer[128];

  int len = snprintf(txBuffer, sizeof(txBuffer), "p:%lu m | v:%lu m/s\r\n",
  						 encoder_position, encoder_velocity);

  	  if (len > 0 && len < sizeof(txBuffer)) {
  	  	  HAL_UART_Transmit_DMA(&huart3, (uint8_t *)txBuffer, len);
  	  } else {

	  // snprintf failed (len < 0) or message truncated (len >= sizeof(txBuffer))
	  dropCount++;

	  // Optional: send a fallback warning (short and safe)
	  const char warn[] = "TX DROP\r\n";
	  HAL_UART_Transmit_DMA(&huart3, (uint8_t *)warn, sizeof(warn) - 1);
  }
  */

	/*
	if (htim->Instance == TIM6)
	  {
	    HAL_TIM_Base_Stop_IT(htim);

	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	  }
	  */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /*
  if (htim->Instance == TIM15)
  {
	  tim15_overflow_count++;
  }

  if (htim->Instance == TIM6) {

	  sensor = get_LTS_value(&cfg);


	  line_bits = ((GPIOC->IDR & LTS_left_GPI_Pin)   ? 4 : 0) |
			 	  ((GPIOC->IDR & LTS_middle_GPI_Pin) ? 2 : 0) |
			 	  ((GPIOC->IDR & LTS_right_GPI_Pin)  ? 1 : 0);

	  g_lts_bits = bits & 0x07;    // publish debounced/raw pattern
  }
  */
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
